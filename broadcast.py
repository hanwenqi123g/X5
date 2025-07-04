#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# 病虫害类型与播报内容的映射
DISEASE_MESSAGES = {
    # 病害类型
    "葡萄霜霉病": "葡萄霜霉病",
    "葡萄灰霉病": "葡萄灰霉病",
    "葡萄白腐病": "葡萄白腐病",
    "葡萄白粉病": "葡萄白粉病",
    "葡萄炭疽病": "葡萄炭疽病",
    "葡萄褐斑病": "葡萄褐斑病",
    "葡萄黑痘病": "葡萄黑痘病",
    "葡萄房枯病": "葡萄房枯病",
    
    # 虫害类型
    "绿盲蝽": "绿盲蝽虫害",
    "葡萄缺节瘿螨": "葡萄缺节瘿螨",
    "葡萄蓟马": "葡萄蓟马",
    
    # 系统指令,
    "reset": "系统复位",
    "normal": "环境状态正常"
}

class DiseaseAnnouncer(Node):
    def __init__(self):
        super().__init__('disease_announcer')
        self.subscription = self.create_subscription(
            String,
            'disease_alert',
            self.listener_callback,
            10)
        self.get_logger().info('葡萄病虫害播报系统已启动...')
        
        # 初始化时播报团队信息
        self.announce("team")

    def listener_callback(self, msg):
        disease_type = msg.data
        self.get_logger().info(f'收到警报: {disease_type}')
        self.announce(disease_type)

    def announce(self, disease_type):
        message = DISEASE_MESSAGES.get(disease_type.lower(), "未知警告类型")
        
        try:
            with serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1
            ) as ser:
                ser.write(message.encode('gbk'))
                self.get_logger().info(f'串口播报: {message}')
                time.sleep(2)  
                
        except serial.SerialException as e:
            self.get_logger().error(f'串口错误: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    announcer = DiseaseAnnouncer()
    
    try:
        rclpy.spin(announcer)
    except KeyboardInterrupt:
        announcer.announce("reset")
    finally:
        announcer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()