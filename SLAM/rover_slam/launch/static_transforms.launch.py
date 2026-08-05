import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # base_link -> camera_link (0.2m forward, 0.3m height)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=['0.2', '0', '0.3', '0', '0', '0', 'base_link', 'camera_link']
        ),
        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0.0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])
