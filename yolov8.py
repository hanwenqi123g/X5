#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_name', 'yolov8n.pt'),
                ('confidence_threshold', 0.5),
                ('image_topic', '/camera/image_raw'),
                ('pub_topic', '/detections'),
                ('display_image', True),
                ('publish_image', False),
                ('device', 'cuda:0') 
            ]
        )
        
        # 获取参数
        model_name = self.get_parameter('model_name').value
        self.conf_thres = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        pub_topic = self.get_parameter('pub_topic').value
        self.display = self.get_parameter('display_image').value
        self.publish_image = self.get_parameter('publish_image').value
        device = self.get_parameter('device').value
        
        # 加载YOLOv8模型
        model_path = os.path.join(
            get_package_share_directory('your_package_name'),
            'models',
            model_name
        )
        self.model = YOLO(model_path)
        self.model.to(device)
        
        # 初始化CV桥
        self.bridge = CvBridge()
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            pub_topic,
            10
        )
        
        if self.publish_image:
            self.image_pub = self.create_publisher(
                Image,
                f'{pub_topic}/annotated',
                10
            )
        
        self.get_logger().info(f'YOLOv8检测器已启动，使用模型: {model_name}')
        self.get_logger().info(f'订阅图像话题: {image_topic}')
        self.get_logger().info(f'发布检测结果到: {pub_topic}')
        self.get_logger().info(f'置信度阈值: {self.conf_thres} | 设备: {device}')

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {str(e)}')
            return

        # 使用YOLOv8进行推理
        results = self.model(
            cv_image, 
            conf=self.conf_thres,
            verbose=False  # 关闭详细输出
        )

        # 准备检测结果消息
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        # 处理检测结果
        for result in results:
            if result.boxes is None:
                continue
                
            for box in result.boxes:
                # 提取检测信息
                conf = float(box.conf.item())
                cls_id = int(box.cls.item())
                cls_name = self.model.names[cls_id]
                
                # 转换边界框坐标
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                width = x2 - x1
                height = y2 - y1
                
                # 填充Detection2D消息
                detection = Detection2D()
                detection.bbox.center.position.x = float(center_x)
                detection.bbox.center.position.y = float(center_y)
                detection.bbox.size_x = float(width)
                detection.bbox.size_y = float(height)
                
                # 填充分类信息
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = cls_name
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)
                
                # 添加到检测数组
                detection_array.detections.append(detection)
                
                # 在图像上绘制结果
                if self.display or self.publish_image:
                    label = f"{cls_name} {conf:.2f}"
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # 发布检测结果
        self.detection_pub.publish(detection_array)
        
        # 发布带标注的图像
        if self.publish_image:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'标注图像发布错误: {str(e)}')
        
        # 显示图像
        if self.display:
            cv2.imshow('YOLOv8 Detection', cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = YOLOv8Detector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()