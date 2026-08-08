import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # Encoder Ticks Pre-Processor
        Node(
            package='rover_slam',
            executable='encoder_ticks_to_odom',
            name='encoder_ticks_to_odom',
            output='screen'
        ),
        # Heuristic Slip Checker Pre-Filter
        Node(
            package='rover_slam',
            executable='heuristic_slip_checker',
            name='heuristic_slip_checker',
            output='screen'
        ),
        # Local EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        )
    ])
