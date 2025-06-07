#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMerger(Node):
    def __init__(self):
        super().__init__('joint_merger')
        self.sub_controller = self.create_subscription(
            JointState, 
            'joint_commands',
            self.controller_callback,
            10)
        self.sub_gui = self.create_subscription(
            JointState,
            'joint_states',
            self.gui_callback,
            10)
        self.pub = self.create_publisher(JointState, 'merged_joint_states', 10)
        
        self.last_controller_msg = None
        self.controller_active = False  # 新增：跟踪控制器状态
    
    def controller_callback(self, msg):
        # 检查是否所有关节位置为零（视为非激活状态）
        self.controller_active = any(pos != 0.0 for pos in msg.position)
        self.last_controller_msg = msg
        self.publish_merged()
    
    def gui_callback(self, msg):
        # 仅在控制器非激活时使用GUI输入
        if not self.controller_active:
            self.publish_merged(msg)
    
    def publish_merged(self, gui_msg=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.controller_active and self.last_controller_msg:
            # 优先使用控制器数据
            msg.name = self.last_controller_msg.name
            msg.position = self.last_controller_msg.position
        elif gui_msg:
            # 回退到GUI输入
            msg.name = gui_msg.name
            msg.position = gui_msg.position
        
        if msg.name:  # 只有有效数据时才发布
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()