#!/usr/bin/env python3
"""
display.launch.py - RViz visualization launch file

This launch file only launches RViz2 for visualization.
Use this alongside Gazebo simulation (rosgp) to visualize the robot.

Note: robot_state_publisher should already be running from the simulation launch file.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get the package share directory
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # RViz2 Node
    # Visualization tool for displaying robot model and TF frames
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_view.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'robot_view.rviz')) else []
    )
    
    # Launch Description
    return LaunchDescription([
        rviz_node
    ])
