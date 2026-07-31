# Autonomous Rover System: Reference & Implementation Guide

This document serves as the living reference for the methodologies and implementation details of the autonomous rover system, focusing on the core hardware modules: Encoders, IMU, and Intel RealSense Depth Camera.

---

## 1. Encoders (Wheel Odometry)

### Methodology
Encoders provide closed-loop feedback on wheel rotation by generating pulses (ticks) as the wheels turn. By counting these ticks over time, we can determine the exact speed and distance traveled by each wheel. Using the rover's specific kinematic model (e.g., differential drive or skid-steer), these wheel velocities are translated into the overall linear and angular velocity of the rover.

### Proposed Implementation
*   **Low-Level Hardware:** Quadrature encoders are attached to the drive motors. A microcontroller (MCU), such as an ESP32 or STM32 (similar to what is typically found in `Low-Level-Software` repositories), uses hardware interrupts to count the ticks without missing pulses.
*   **Communication:** The MCU calculates the raw wheel velocities and sends them over UART, SPI, or CAN bus to the main computation unit (e.g., Jetson or Raspberry Pi).
*   **High-Level Software:** A driver node receives this data and publishes standard ROS messages (`nav_msgs/Odometry`). It also broadcasts a Transform (`/tf`) from the `odom` frame to the `base_link` (the robot's center).

---

## 2. IMU (Inertial Measurement Unit)

### Methodology
An IMU measures the rover's 3-axis linear acceleration and 3-axis angular velocity. While trying to calculate position purely from IMU acceleration leads to rapid drift, the IMU is highly reliable for determining short-term orientation (especially heading/yaw).

### Proposed Implementation
*   **Hardware Integration:** The IMU (e.g., BNO085, MPU6050, or an industrial grade IMU) is connected via I2C/SPI either to the MCU or directly to the main computer. 
*   **Filtering:** A filter algorithm (like Madgwick or Mahony) runs to compute reliable orientation quaternions from the raw accelerometer and gyroscope data.
*   **Sensor Fusion (The EKF):** Because wheel odometry suffers from slippage (e.g., wheels spinning in sand) and IMUs suffer from drift, we implement an Extended Kalman Filter (EKF). Using a package like `robot_localization`, we fuse the encoder data (good for continuous translation) with the IMU data (good for orientation) to produce a highly accurate, robust local state estimate (`/odometry/filtered`).

---

## 3. Perception Module (Intel RealSense)

### Role & Requirements for ERC
In the European Rover Challenge (ERC), the rover navigates a "Mars Yard" filled with obstacles like rocks, craters, and steep slopes. It also frequently needs to locate AR tags (ArUco markers) or specific probes.
Therefore, the perception module must:
1. **Detect Obstacles:** Identify rocks/craters and mark them as impassable in a costmap.
2. **Provide Global Odometry (Optional but highly recommended):** Since GPS is unavailable indoors or heavily restricted, Visual SLAM uses camera features to correct the continuous drift of wheel odometry.
3. **Detect Markers:** Find and calculate the 3D position (pose) of ArUco navigation waypoints relative to the rover.

### Inputs
The Perception Module ingests raw sensor data, primarily from the `realsense2_camera` driver node:
*   **RGB Image (`sensor_msgs/Image`):** The standard color video stream.
*   **Depth Data (`sensor_msgs/Image` or `sensor_msgs/PointCloud2`):** Either an image where pixel values represent distance, or a dense 3D cloud of points. We typically configure the RealSense to provide "Aligned Depth to Color" so every color pixel has an exact depth value.
*   **Camera Intrinsics (`sensor_msgs/CameraInfo`):** Calibration data required to project 2D pixels into 3D space.

### Outputs
The module processes these inputs and outputs actionable intelligence for the Navigation Stack and the Mission Supervisor:
*   **2D LaserScan (`sensor_msgs/LaserScan`):** A flat 2D slice of the obstacles, converted from the 3D depth data.
*   **Map Transform (`/tf` from `map` to `odom`):** The global position correction provided by Visual SLAM.
*   **Marker Poses (`geometry_msgs/PoseStamped`):** The coordinates of any detected ArUco markers.

### The Best & Easiest Implementation for ERC
Writing custom point-cloud processing code from scratch is notoriously difficult and computationally expensive. For the ERC, the most robust and accessible approach relies heavily on standard, proven ROS packages:

1.  **The Camera Driver:** Use the official `realsense2_camera` node. Ensure you enable `align_depth:=true` in the launch file.
2.  **Obstacle Detection (The 2D Trick):** Do *not* try to navigate using a dense 3D Voxel Grid directly unless you have immense computing power. **The easiest method** is to use the `depthimage_to_laserscan` or `pointcloud_to_laserscan` package. 
    *   *Why?* It compresses the heavy 3D data into a lightweight 2D slice (like a 2D LiDAR). You configure it to only look at a specific height range (e.g., from 10cm off the ground up to the rover's height). Anything in that slice is an obstacle. The Nav2/`move_base` stack handles 2D laser scans effortlessly.
3.  **Visual SLAM:** Use `RTAB-Map`. It is deeply integrated with the RealSense ecosystem. By feeding it the RGB image, Aligned Depth image, and Camera Info, `RTAB-Map` will automatically generate the 3D map and handle loop closures without you needing to write any complex feature-matching code.
4.  **Marker Detection:** Use the `aruco_ros` package. Feed it your RGB image and it will spit out the exact 3D coordinates of the navigation tags, which your state machine can then send directly to the navigation stack as goals.

---

*Note: This document will be updated as we finalize design decisions and proceed with the implementation of each module.*
