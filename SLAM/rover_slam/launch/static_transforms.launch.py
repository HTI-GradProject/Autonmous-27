import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # base_link -> camera_link (Camera mounted 0.2m forward, 0.3m high on rover chassis)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '--x', '0.2', '--y', '0.0', '--z', '0.3',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),

        # base_link -> imu_link (BNO055 IMU mounted at chassis center, 0.1m high)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ]
        ),

        # camera_link -> camera_depth_optical_frame (ROS REP-103 standard Optical Frame rotation)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_optical',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '-1.57079632679', '--pitch', '0.0', '--yaw', '-1.57079632679',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_depth_optical_frame'
            ]
        )
    ])

