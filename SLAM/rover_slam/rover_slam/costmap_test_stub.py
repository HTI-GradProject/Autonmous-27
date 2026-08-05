#!/usr/bin/env python3
"""
costmap_test_stub.py
Publishes simulated rock obstacle point clouds to /perception/obstacles_only
to verify nav2_costmap_2d inflation output independently.
Developer Track: Person 2 (Task 5B)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CostmapTestStubNode(Node):
    def __init__(self):
        super().__init__('costmap_test_stub')
        self.get_logger().info('Costmap Test Stub Node Initialized.')
        # TODO: Implement synthetic obstacle PointCloud2 publisher

def main(args=None):
    rclpy.init(args=args)
    node = CostmapTestStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
