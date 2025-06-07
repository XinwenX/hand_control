#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import JointState
import numpy as np

class HandController(Node):
    def __init__(self):
        super().__init__('hand_controller')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)  # 启用DEBUG日志
        # 串口配置 - 根据您的实际设置调整
        self.serial_port = serial.Serial(
            '/dev/ttyUSB0',  # 可能是ttyACM0
            baudrate=115200,
            timeout=1
        )
        if not self.serial_port.is_open:
            self.get_logger().error(f"Failed to open {self.serial_port.port}")
        else:
            self.get_logger().info(f"Connected to {self.serial_port.port}")

        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.get_logger().debug(f"Publisher created for topic: {self.joint_pub.topic}")

        self.timer = self.create_timer(0.1, self.read_serial)  # 10Hz
        self.active = False
        
        # 灵巧手的关节名称
        self.joint_names = [
            'L_thumb_proximal_yaw_joint',
            'L_thumb_proximal_pitch_joint',
            'L_index_proximal_joint',
            'L_middle_proximal_joint',
            'L_ring_proximal_joint',
            'L_pinky_proximal_joint'
        ]
        
        # 初始发布全零位置（非激活状态）
        self.publish_zero_positions()
        self.timer = self.create_timer(0.1, self.read_serial)
    
    def publish_zero_positions(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        self.joint_pub.publish(msg)
    
    def read_serial(self):
        if self.serial_port.in_waiting:
            try:
                raw_data = self.serial_port.readline()
                # self.get_logger().debug(f"Raw data: {raw_data}")  # 打印原始数据
                line = raw_data.decode('utf-8').strip()
                # self.get_logger().debug(f"Decoded line: '{line}'")  # 打印解码后数据
                
                # 处理控制指令
                if line == "Activated":
                    self.active = True
                    self.get_logger().info("Controller activated")
                    return
                elif line == "Deactivated":
                    self.active = False
                    self.publish_zero_positions()  # 复位关节位置
                    self.get_logger().info("Controller deactivated")
                    return
                
                # 仅在激活状态处理数据
                if self.active:
                    values = [int(x) for x in line.split(',') if x.isdigit()]

                    if len(values) == 6:
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.name = self.joint_names
                        msg.position = [
                            self.map_value(np.clip(values[0],150, 810), 810, 150, -0.1, 1.3),
                            self.map_value(np.clip(values[1],170, 810), 810, 170, 0.0, 0.5),
                            self.map_value(values[2], 665, 360, 0.0, 1.7),
                            self.map_value(values[3], 665, 360, 0.0, 1.7),
                            self.map_value(values[4], 665, 360, 0.0, 1.7),
                            self.map_value(values[5], 665, 360, 0.0, 1.7)
                        ]
                        self.get_logger().debug(f"Prepared message: names={msg.name}, positions={msg.position}")
                        self.joint_pub.publish(msg)
                        # self.get_logger().debug("Message published")  # 确认执行到此处
                        
            except ValueError as e:
                self.get_logger().warn(f"Ignored non-numeric data: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial error: {str(e)}")
    
    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = HandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()