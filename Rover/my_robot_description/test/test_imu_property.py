#!/usr/bin/env python3
"""
test_imu_property.py - Property test for IMU data completeness

Property 3: IMU Data Completeness
Validates: Requirements 3.3, 3.4

This test verifies that all IMU messages contain:
- linear_acceleration with non-zero covariance
- angular_velocity with non-zero covariance
- Complete data fields

Runs minimum 100 iterations as specified in requirements.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time


class IMUPropertyTest(unittest.TestCase):
    """Property-based test for IMU data completeness"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2"""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2"""
        rclpy.shutdown()
    
    def test_imu_data_completeness(self):
        """
        Property: All IMU messages must contain complete data
        - linear_acceleration (x, y, z)
        - angular_velocity (x, y, z)
        - non-zero covariance matrices
        
        Collects 100 messages and verifies each one.
        """
        
        class IMUCollector(Node):
            def __init__(self):
                super().__init__('imu_collector')
                self.messages = []
                self.target_count = 100
                self.subscription = self.create_subscription(
                    Imu,
                    '/imu/data',
                    self.callback,
                    10
                )
                
            def callback(self, msg):
                if len(self.messages) < self.target_count:
                    self.messages.append(msg)
        
        # Create collector node
        collector = IMUCollector()
        
        # Collect messages (with timeout)
        start_time = time.time()
        timeout = 30.0  # 30 seconds timeout
        
        while len(collector.messages) < collector.target_count:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail(f"Timeout: Only collected {len(collector.messages)} messages in {timeout}s")
        
        # Verify each message
        for i, msg in enumerate(collector.messages):
            with self.subTest(message=i):
                # Check linear_acceleration exists and has values
                self.assertIsNotNone(msg.linear_acceleration)
                self.assertIsInstance(msg.linear_acceleration.x, float)
                self.assertIsInstance(msg.linear_acceleration.y, float)
                self.assertIsInstance(msg.linear_acceleration.z, float)
                
                # Check angular_velocity exists and has values
                self.assertIsNotNone(msg.angular_velocity)
                self.assertIsInstance(msg.angular_velocity.x, float)
                self.assertIsInstance(msg.angular_velocity.y, float)
                self.assertIsInstance(msg.angular_velocity.z, float)
                
                # Check covariance matrices are not all zeros
                linear_accel_cov = msg.linear_acceleration_covariance
                angular_vel_cov = msg.angular_velocity_covariance
                
                self.assertEqual(len(linear_accel_cov), 9, "Linear acceleration covariance must have 9 elements")
                self.assertEqual(len(angular_vel_cov), 9, "Angular velocity covariance must have 9 elements")
                
                # At least one covariance value should be non-zero
                self.assertTrue(
                    any(abs(v) > 1e-10 for v in linear_accel_cov),
                    f"Message {i}: Linear acceleration covariance is all zeros"
                )
                self.assertTrue(
                    any(abs(v) > 1e-10 for v in angular_vel_cov),
                    f"Message {i}: Angular velocity covariance is all zeros"
                )
        
        # Cleanup
        collector.destroy_node()
        
        print(f"\n✓ Property test passed: All {len(collector.messages)} IMU messages contain complete data")


if __name__ == '__main__':
    unittest.main()
