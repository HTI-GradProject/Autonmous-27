#!/usr/bin/env python3
"""
encoder_ticks_to_odom.py
Converts raw wheel encoder ticks into nav_msgs/Odometry Twist velocity messages.
Developer Track: Person 1 (Task 2A.1)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class EncoderTicksToOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_ticks_to_odom')
        self.get_logger().info('Encoder Ticks to Odometry Node Initialized.')
        # TODO: Implement wheel tick subscription & differential drive math

def main(args=None):
    rclpy.init(args=args)
    node = EncoderTicksToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
