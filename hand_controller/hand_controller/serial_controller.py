#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import JointState

class HandController(Node):
    def __init__(self):
        super().__init__('hand_controller')
        # 串口配置 - 根据您的实际设置调整
        self.serial_port = serial.Serial(
            '/dev/ttyUSB0',  # 可能是ttyACM0
            baudrate=9600,
            timeout=1
        )
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.read_serial)  # 10Hz
        
        # 灵巧手的关节名称
        self.joint_names = [
            'L_thumb_proximal_yaw_joint',
            'L_thumb_proximal_pitch_joint',
            'L_index_proximal_joint',
            'L_middle_proximal_joint',
            'L_ring_proximal_joint',
            'L_pinky_proximal_joint'
        ]
        
    def read_serial(self):
        if self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                values = [int(x) for x in line.split(',')]
                
                if len(values) == 6:
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = self.joint_names
                    # 将0-1023映射到关节角度范围
                    msg.position = [
                        self.map_value(values[0], 0, 1023, -0.1, 1.3),    # 拇指偏航
                        self.map_value(values[1], 0, 1023, 0.0, 0.5),     # 拇指俯仰
                        self.map_value(values[2], 0, 1023, 0.0, 1.7),     # 食指
                        self.map_value(values[3], 0, 1023, 0.0, 1.7),     # 中指
                        self.map_value(values[4], 0, 1023, 0.0, 1.7),     # 无名指
                        self.map_value(values[5], 0, 1023, 0.0, 1.7)      # 小指
                    ]
                    self.joint_pub.publish(msg)
                    
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