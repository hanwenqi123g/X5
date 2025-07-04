#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from threading import Lock
from std_msgs.msg import String  
# 病害严重程度映射表
DISEASE_SEVERITY = {
    "葡萄霜霉病": [1, 2, 3, 4, 5],
    "葡萄灰霉病": [1, 2, 3, 4, 5],
    "葡萄白腐病": [1, 2, 4, 5, 6],
    "葡萄白粉病": [1, 2, 3, 4, 5],
    "葡萄炭疽病": [1, 3, 4, 5, 6],
    "葡萄褐斑病": [1, 2, 3, 4, 5],
    "葡萄黑痘病": [1, 2, 3, 4, 6],
    "葡萄房枯病": [1, 2, 4, 5, 6]
}

class IrrigationController(Node):
    def __init__(self):
        super().__init__('irrigation_controller')
        
        # ROS2参数配置
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        # 初始化串口
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.ser = None
        self.serial_lock = Lock()
        self.init_serial()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            '/disease_info',
            self.disease_callback,
            10)
        
        self.get_logger().info("喷灌控制器已启动，等待病害信息...")
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            self.get_logger().info(f"成功连接串口: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {str(e)}")
    
    def open_relay(self):
        """打开继电器(启动喷灌)"""
        with self.serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    command = bytes.fromhex('A0 01 01 A2')
                    self.ser.write(command)
                    self.get_logger().info("继电器已打开 - 喷灌开始")
            except Exception as e:
                self.get_logger().error(f"打开继电器错误: {str(e)}")
    
    def close_relay(self):
        """关闭继电器(停止喷灌)"""
        with self.serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    command = bytes.fromhex('A0 01 00 A1')
                    self.ser.write(command)
                    self.get_logger().info("继电器已关闭 - 喷灌结束")
            except Exception as e:
                self.get_logger().error(f"关闭继电器错误: {str(e)}")
    
    def disease_callback(self, msg):
        """处理接收到的病害信息"""
        try:
            # 解析消息格式: "病害类型:严重程度"
            disease, severity_str = msg.data.split(':')
            severity = int(severity_str)
            
            self.get_logger().info(f"收到病害信息: {disease}, 严重程度: {severity}")
            
            # 获取喷灌方案
            spray_times = self.get_spray_times(disease, severity)
            
            if spray_times > 0:
                self.execute_irrigation(spray_times)
            else:
                self.get_logger().warn("无需喷灌: 严重程度为0或病害未知")
                
        except ValueError:
            self.get_logger().error("无效的消息格式! 应为 '病害类型:严重程度'")
        except Exception as e:
            self.get_logger().error(f"处理病害信息时出错: {str(e)}")
    
    def get_spray_times(self, disease, severity):
        """根据病害类型和严重程度确定喷灌次数"""
        if disease not in DISEASE_SEVERITY:
            self.get_logger().warn(f"未知病害类型: {disease}")
            return 0
        
        levels = DISEASE_SEVERITY[disease]
        # 确保严重程度在有效范围内
        clamped_severity = max(1, min(severity, len(levels)))
        return levels[clamped_severity - 1]
    
    def execute_irrigation(self, spray_times):
        """执行喷灌操作"""
        self.get_logger().info(f"开始喷灌，计划喷灌次数: {spray_times}")
        
        for i in range(spray_times):
            self.get_logger().info(f"喷灌周期 #{i+1}/{spray_times}")
            
            # 打开继电器喷水
            self.open_relay()
            time.sleep(3)  # 喷水持续时间
            
            # 关闭继电器停止喷水
            self.close_relay()
            
            # 如果不是最后一次，添加间隔
            if i < spray_times - 1:
                time.sleep(2)  # 喷水间隔时间
        
        self.get_logger().info("喷灌计划完成")

def main(args=None):
    rclpy.init(args=args)
    controller = IrrigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理工作
        if controller.ser and controller.ser.is_open:
            controller.close_relay()  # 确保继电器关闭
            controller.ser.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()