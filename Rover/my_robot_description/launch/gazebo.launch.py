#!/usr/bin/env python3
"""
gazebo.launch.py - Gazebo simulation launch file

This launch file:
1. Processes the xacro file into URDF
2. Launches Gazebo Ignition with empty world
3. Launches robot_state_publisher with use_sim_time=true
4. Spawns robot at origin using spawn_entity service

Use this to test the robot in Gazebo simulation.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import re

def launch_setup(context, *args, **kwargs):
    # Retrieve the world launch argument
    world_file = LaunchConfiguration('world').perform(context)
    
    # Determine the world file path and extract the world name from the SDF/World file
    pkg_worlds = FindPackageShare('worlds').find('worlds')
    if os.path.isabs(world_file):
        world_path = world_file
    else:
        world_path = os.path.join(pkg_worlds, 'worlds', world_file)
        
    world_name = "rover_world"
    if "empty" in world_file:
        world_name = "empty"
    elif os.path.exists(world_path):
        try:
            with open(world_path, 'r') as f:
                content = f.read()
                match = re.search(r'<world\s+name=["\']([^"\']+)["\']>', content)
                if match:
                    world_name = match.group(1)
        except Exception as e:
            print(f"[gazebo.launch] Error reading world file: {e}")

    # Get the my_robot_description package share directory
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    
    # Process the xacro file to generate URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )
    
    # Gazebo Launch using the worlds package's custom launch file (which sets up model resource paths)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
        ]),
        launch_arguments={
            'world': world_file
        }.items()
    )
    
    # Spawn Robot Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'  # Spawn slightly higher to avoid colliding with terrain features
        ],
        output='screen'
    )
    
    # Bridge between Ignition Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloud2',
        ],
        remappings=[
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
        ],
        output='screen'
    )
    
    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge
    ]


def generate_launch_description():
    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world1.world',
        description='Name of the world file or path to load (e.g., world1.world or empty_with_sensors.sdf)'
    )
    
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])

