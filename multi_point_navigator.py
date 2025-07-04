#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MultiPointNavigator(Node):
    def __init__(self):
        super().__init__('multi_point_navigator')
        
        # 参数配置
        self.declare_parameter('goal_tolerance', 0.3)  # 目标点容差 (米)
        self.declare_parameter('wait_time', 2.0)       # 到达后等待时间 (秒)
        self.declare_parameter('loop_navigation', False)  # 是否循环导航
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.wait_time = self.get_parameter('wait_time').value
        self.loop = self.get_parameter('loop_navigation').value
        
        # 创建动作客户端 (导航到目标点)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("等待导航动作服务器...")
        self.nav_client.wait_for_server()
        self.get_logger().info("导航动作服务器已连接!")
        
        # 定义目标点列表 (x, y, z, qx, qy, qz, qw)
        self.goal_points = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],   # 起点
            [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],   # 点1
            [2.0, 2.0, 0.0, 0.0, 0.0, 0.707, 0.707],  # 点2
            [0.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0],   # 点3
            [0.0, 0.0, 0.0, 0.0, 0.0, -0.707, 0.707]  # 回到起点
        ]
        
        # 当前目标索引
        self.current_goal_index = 0
        
        # 状态标志
        self.navigating = False
        self.goal_reached = False
        self.waiting_for_continue = False
        
        # 创建发布者 (完成通知)
        self.done_pub = self.create_publisher(Empty, '/done', 10)
        
        # 创建订阅者 (继续指令)
        self.continue_sub = self.create_subscription(
            Empty, 
            '/continue', 
            self.continue_callback, 
            10
        )
        
        # 创建状态发布者
        self.status_pub = self.create_publisher(String, '/nav_status', 10)
        
        # 创建定时器
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # 开始第一个目标点导航
        self.navigate_to_next_point()
    
    def continue_callback(self, msg):
        """接收到继续指令时的回调"""
        if self.waiting_for_continue:
            self.get_logger().info("接收到继续指令，前往下一个目标点")
            self.waiting_for_continue = False
            self.navigate_to_next_point()
    
    def navigate_to_next_point(self):
        """导航到下一个目标点"""
        if self.current_goal_index >= len(self.goal_points):
            if self.loop:
                self.current_goal_index = 1  # 跳过起点重新开始
                self.get_logger().info("所有目标点已完成，开始新一轮导航...")
            else:
                self.get_logger().info("所有目标点已完成!")
                self.publish_status("任务完成")
                return
        
        # 获取下一个目标点
        goal_data = self.goal_points[self.current_goal_index]
        goal_pose = self.create_pose_stamped(
            goal_data[0], goal_data[1], goal_data[2],
            goal_data[3], goal_data[4], goal_data[5], goal_data[6]
        )
        
        # 发送导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"导航到目标点 {self.current_goal_index}: "
                              f"({goal_data[0]:.2f}, {goal_data[1]:.2f})")
        self.publish_status(f"导航中: 目标点 {self.current_goal_index}")
        
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.navigating = True
        self.goal_reached = False
    
    def goal_response_callback(self, future):
        """导航目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            self.publish_status("目标被拒绝")
            return
        
        self.get_logger().info('目标已接受，正在导航...')
        
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # 检查是否到达目标点
        goal = self.goal_points[self.current_goal_index]
        distance = math.sqrt(
            (current_pose.position.x - goal[0])**2 +
            (current_pose.position.y - goal[1])**2
        )
        
        if distance < self.goal_tolerance and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(f"已到达目标点 {self.current_goal_index}!")
            self.publish_status(f"到达目标点 {self.current_goal_index}")
            
            # 发布完成消息
            self.done_pub.publish(Empty())
            
            # 等待继续指令
            self.waiting_for_continue = True
            self.get_logger().info(f"等待继续指令... (等待时间: {self.wait_time}秒)")
            self.publish_status(f"等待继续指令")
            
            # 设置超时自动继续
            self.create_timer(self.wait_time, self.timeout_continue)
    
    def timeout_continue(self):
        """超时自动继续"""
        if self.waiting_for_continue:
            self.get_logger().info("等待超时，自动继续")
            self.waiting_for_continue = False
            self.current_goal_index += 1
            self.navigate_to_next_point()
    
    def nav_result_callback(self, future):
        """导航结果回调"""
        result = future.result().result
        self.navigating = False
        self.get_logger().info(f'导航完成结果: {result.error_code}')
        
        # 如果导航失败但已到达目标点，仍然认为成功
        if result.error_code != 4 and not self.goal_reached:
            self.get_logger().error(f'导航失败! 错误代码: {result.error_code}')
            self.publish_status(f"导航失败: 目标点 {self.current_goal_index}")
        else:
            # 如果到达目标点但未收到继续指令，增加索引
            if self.goal_reached and not self.waiting_for_continue:
                self.current_goal_index += 1
    
    def timer_callback(self):
        """定时器回调函数"""
        # 如果没有在导航且没有等待继续指令，尝试下一个目标点
        if not self.navigating and not self.waiting_for_continue:
            self.navigate_to_next_point()
    
    def create_pose_stamped(self, x, y, z, qx, qy, qz, qw):
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose
    
    def publish_status(self, message):
        """发布状态消息"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
    
    def calculate_quaternion(self, yaw):
        """根据偏航角计算四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return 0.0, 0.0, sy, cy

def main(args=None):
    rclpy.init(args=args)
    navigator = MultiPointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()