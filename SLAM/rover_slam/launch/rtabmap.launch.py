import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    rtabmap_config_path = os.path.join(pkg_share, 'config', 'rtabmap.yaml')

    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_config_path],
            remappings=[
                ('odom', '/odometry/filtered'),
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/filtered'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('landmark', '/perception/aruco_pose')
            ]
        )
    ])
