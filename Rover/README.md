# Rover Simulation & Configuration

This directory contains packages defining the Mars Rover style robot model, simulation worlds, and marsyard elements.

## Subdirectories

*   **[my_robot_description](file:///home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Rover/my_robot_description)**: URDF xacro description and ROS 2 launch system for spawning the rover.
*   **[worlds](file:///home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Rover/worlds)**: Defines environments (Mars Yard and empty worlds) and provides helper launch scripts.
*   **[marsyard](file:///home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Rover/marsyard)**: Model definitions and configurations representing physical Mars Yard terrains.

---

## Active Rover Topics (Gazebo Simulation)

When the simulation launch command (`ros2 launch my_robot_description gazebo.launch.py`) is running, the following ROS 2 topics are active for interfacing with the rover:

### 1. Actuation & Motion Control
*   **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
    *   *Direction:* Subscriber (Input to Gazebo)
    *   *Description:* Send linear/angular velocity commands to drive the rover.

### 2. State & Inertial Feedback
*   **`/odom`** (`nav_msgs/msg/Odometry`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Calculates raw wheel odometry computed from model dynamics.
*   **`/imu/data`** (`sensor_msgs/msg/Imu`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Publishes rover linear accelerations and angular velocities with realistic Gaussian noise.

### 3. Integrated Intel RealSense D435i Camera
*   **`/camera/image_raw`** (`sensor_msgs/msg/Image`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Standard RGB video frame feed from the front-mounted camera.
*   **`/camera/camera_info`** (`sensor_msgs/msg/CameraInfo`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Lens camera intrinsics calibration parameters.
*   **`/camera/depth/image_raw`** (`sensor_msgs/msg/Image`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Aligned raw depth image feed.
*   **`/camera/depth/color/points`** (`sensor_msgs/msg/PointCloud2`)
    *   *Direction:* Publisher (Output from Gazebo)
    *   *Description:* Dense 3D Point Cloud ($x, y, z$) mapping spatial points relative to the camera frame. **(Primary input to the Terrain Geometry node)**.
