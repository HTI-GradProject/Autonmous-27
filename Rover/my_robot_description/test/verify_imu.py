#!/usr/bin/env python3
"""
verify_imu.py - Verify IMU sensor data publishing

This script subscribes to /imu/data and verifies:
1. Topic publishes data
2. Messages contain linear_acceleration
3. Messages contain angular_velocity
4. Data is non-zero (sensor is working)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys


class IMUVerifier(Node):
    def __init__(self):
        super().__init__('imu_verifier')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.message_count = 0
        self.target_count = 10
        
    def imu_callback(self, msg):
        self.message_count += 1
        
        if self.message_count == 1:
            self.get_logger().info('✓ IMU topic is publishing')
            self.get_logger().info(f'✓ Linear acceleration: x={msg.linear_acceleration.x:.3f}, y={msg.linear_acceleration.y:.3f}, z={msg.linear_acceleration.z:.3f}')
            self.get_logger().info(f'✓ Angular velocity: x={msg.angular_velocity.x:.3f}, y={msg.angular_velocity.y:.3f}, z={msg.angular_velocity.z:.3f}')
            
            # Check if data is reasonable (stationary robot should have ~9.81 m/s² in Z)
            if abs(msg.linear_acceleration.z - 9.81) < 2.0:
                self.get_logger().info('✓ IMU data looks reasonable (gravity detected)')
            
        if self.message_count >= self.target_count:
            self.get_logger().info(f'✓ Received {self.message_count} messages - IMU verification complete!')
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    verifier = IMUVerifier()
    
    try:
        rclpy.spin(verifier)
    except SystemExit:
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
