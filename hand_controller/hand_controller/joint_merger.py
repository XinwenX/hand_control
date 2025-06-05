#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMerger(Node):
    def __init__(self):
        super().__init__('joint_merger')
        self.sub1 = self.create_subscription(
            JointState, 
            'joint_commands',
            self.controller_callback,
            10)
        self.sub2 = self.create_subscription(
            JointState,
            'joint_states',
            self.gui_callback,
            10)
        self.pub = self.create_publisher(JointState, 'merged_joint_states', 10)
        
        self.last_gui_msg = None
        self.last_controller_msg = None
        
    def controller_callback(self, msg):
        self.last_controller_msg = msg
        self.publish_merged()
        
    def gui_callback(self, msg):
        self.last_gui_msg = msg
        self.publish_merged()
        
    def publish_merged(self):
        if self.last_gui_msg and self.last_controller_msg:
            merged = JointState()
            merged.header.stamp = self.get_clock().now().to_msg()
            merged.name = self.last_controller_msg.name
            merged.position = self.last_controller_msg.position
            self.pub.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = JointMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()