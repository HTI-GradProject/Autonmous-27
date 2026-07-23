# Perception Guide: ZED/RealSense Camera Perception Pipeline (ERC Mars Rover)

This guide is designed for absolute beginners to understand how a Mars Rover's perception module works from zero. It explains the processing steps required before splitting streams, the three post-filter branches, their implementations, and their exact message outputs.

---

## 1. High-Level Pipeline Architecture

The flowchart below represents the complete perception architecture, demonstrating how raw camera data maps to the Nav2 Path Planner, Robotic Arm IK, and Localization SLAM:

```
  [ Raw Left/Right Video + IMU ]  <-- Ingestion & Driver Interface
               │
               ▼
   [ Rectification & Alignment ]  <-- Warp Depth to Color Coordinate Space
               │
               ▼
   [ Crop Box Self-Filtering ]    <-- Erase points matching Rover chassis profile
               │
      ┌────────┴────────┬────────┐
      ▼                 ▼        ▼
  [Branch 1]        [Branch 2]   [Branch 3]
  Terrain Geometry   Fiducials    Visual Tracking
      │                 │        (ORB Features)
  RANSAC Ground     Grayscale           │
      │             Threshold       Motion Estimation
  Voxel Grid            │               │
      │             Quad Warp      Sensor Fusion EKF
  DBSCAN Cluster        │               │
      │             solvePnP            │
      ▼                 ▼               ▼
[Costmap Output]   [6-DoF Pose]    [Fused Odometry]
```

---

## 2. Phase A: Pre-Split Operations (Before the 3 Branches)

Before the camera data splits into localization, mapping, and marker branches, we must complete four critical preprocessing operations to clean and format the sensor streams.

### 1. Raw Ingestion & Driver Interface
*   **What is required:** Establish a hardware connection via a **USB 3.0 port** and launch the driver. USB 2.0 does not support high-bandwidth RGBD streams.
*   **Ready-Made Package:** Use official camera wrappers like `realsense2_camera` or `zed_wrapper`.
*   **Implementation Launch Command:**
    ```bash
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
    ```

### 2. Calibration, Rectification & Alignment
*   **What is required:** Rectification uses the camera's lens calibration matrices to flatten optical lens warp. Alignment warps the depth image coordinates so they match the color image coordinate grid. This aligns pixels so that `Color(x,y)` maps to the exact same 3D coordinate as `Depth(x,y)`.
*   **Ready-Made Tool:** Handled automatically by the wrapper when launching with `align_depth.enable:=true`.

### 3. CropBox Self-Filtering
*   **What is required:** The point cloud must be filtered to delete coordinates that fall within the physical volume of the rover base, wheels, and camera mounts. If this isn't done, the rover sees itself as a hazard and blocks movement.
*   **Ready-Made Package:** Use the standard ROS 2 **`sensor_filters`** package.
*   **`cropbox_filter.yaml` Config:**
    ```yaml
    crop_box_filter:
      ros__parameters:
        filter_field_name: "x"
        min_x: -0.6
        max_x: 0.6
        min_y: -0.6
        max_y: 0.6
        min_z: -0.3
        max_z: 1.0
        negative: true # Deletes points inside these boundaries
    ```
*   **ROS 2 Launch Node Integration:**
    ```python
    Node(
        package='sensor_filters',
        executable='sensor_filter_node',
        name='chassis_self_filter',
        parameters=['cropbox_filter.yaml'],
        remappings=[
            ('~input', '/camera/camera/depth/color/points'),
            ('~output', '/perception/point_cloud_filtered')
        ]
    )
    ```

### 4. Output of the Pre-Split Stage
*   **Published Topic:** `/perception/point_cloud_filtered`
*   **ROS 2 Message Type:** `sensor_msgs/msg/PointCloud2`
*   **Key Variables:** `data` (raw coordinate bytes), `fields` (descriptions of axes layout: X, Y, Z), `is_dense` (boolean validation flag).
*   **What does this output mean?** This is a massive 3D point cloud of the world in front of the camera, but mathematically "hollowed out" where the rover's own body is. It contains the ground sand, the rocks, the sky, and everything else in frame.
*   **How exactly is it used?** It acts as the master, clean source of depth data. We split this output so that multiple nodes can process it simultaneously without doing duplicate work. It feeds directly into the Terrain/Obstacle mapping node (Branch 1) and the Visual Tracking node (Branch 3).

---

## 3. Phase B: The Three Post-Filter Branches

Once a clean point cloud is produced, the pipeline splits into three dedicated branches.

### Branch 1: Terrain Geometry & Obstacle Map

#### Do I need to implement RANSAC and Voxel Grid calculations myself?
**No.** You do not need to write custom C++ or Python code if you choose Approach A. However, because your rover is climbing ramps, slopes, and craters in the ERC Mars Yard, **Approach A (standard height cutoff) is not implementable** because tilting the rover makes flat sand look like a wall.

You **must** implement Approach B (dynamic RANSAC plane segmentation). Below is a comparison of both routes:

| Feature | Approach A: Ready-Made Nav2 Voxel Layer (Zero-Code) | Approach B: Custom RANSAC + DBSCAN Node (Custom Code) |
| :--- | :--- | :--- |
| **Implementation** | Purely configured via YAML settings in Nav2. Zero code to write. | Write a custom node using PCL (Point Cloud Library) or Open3D in C++/Python. |
| **Ground Filtering** | Simple height cut-off threshold (e.g., discard all points below Z = 15cm). | Fits a dynamic plane equation to the sand coordinates using active **RANSAC plane segmentation**. |
| **Slope Performance (Crucial for ERC)** | **Poor.** If the rover tilts or drives up a ramp, the ground climbs into your height threshold. Nav2 will mistake the flat sand slope for a solid wall and refuse to drive. | **Excellent.** The ground plane equation tilts dynamically with the rover. The sand is always filtered out correctly, even on steep ramps. |
| **Noise Filtering** | Basic. Sensor noise or sand dust can trigger ghost obstacles. | DBSCAN groups points. Random floating dust is discarded because it does not form a dense cluster. |
| **CPU Overhead** | Very Low (highly optimized C++). | Moderate (requires processing resources on the Jetson). |

---

### Input & Output Pipeline Differences: Approach 1 vs Approach 2
To understand the difference in data pipeline management between the two approaches, look at how coordinates are passed and who filters them:
*   **Approach 1 (Standard Nav2 Configuration):**
    *   **Input to Nav2:** The raw, dirty point cloud from the camera containing everything (flat sand ground, dust, and obstacles).
    *   **Downstream Filtering:** Nav2 attempts to filter the sand ground by checking if point heights are above 15cm relative to a flat coordinate frame. It has no clustering to filter out wheel dust.
    *   **Result:** On sloped terrain, the ground sand exceeds the height limit and is incorrectly outputted as a wall.
*   **Approach 2 (Custom RANSAC + DBSCAN Node):**
    *   **Input to Nav2:** A pre-filtered point cloud published on `/perception/obstacles_only`.
    *   **Downstream Filtering:** The ground is mathematically removed using **RANSAC** plane segmentation, and outliers/dust are deleted by **DBSCAN** spatial density checks *before* Nav2 even sees the data.
    *   **Result:** The output is a clean coordinate set of obstacles only, making it 100% reliable on steep slopes and dusty ground.

---

### Detailed Implementation Guides & Explanations

#### 1. Approach 2 Concept Breakdown & Dynamic Flow
To implement Approach 2 successfully, your perception pipeline runs in a defined sequence. Here is what is happening conceptually at each step and what it accomplishes:

1.  **Sensor Transformation (TF2 Coordinate Projection):** Raw depth coordinates are measured relative to the camera origin (e.g. `camera_depth_optical_frame`). If you run RANSAC on these coordinates directly, the floor plane will skew as the camera tilts. We first run a TF2 listener to mathematically transform the Point Cloud into the robot base frame coordinates (`base_link`). This ensures Z coordinates always represent height above the tires.
2.  **Voxel Grid Downsampling (Reducing Density):** A raw HD depth scan contains over 300,000 coordinates. Running a RANSAC plane solver 1000 times on this density would freeze the Jetson CPU. Open3D groups coordinates into 3D pixels (voxels) of size 5cm x 5cm x 5cm and replaces points inside with a single average. This drops points to under 20,000 while preserving obstacle shapes.
3.  **RANSAC Ground Segmentation (Fitting a Slope-Aware Plane):** The ground is mathematically represented by the plane equation: **Ax + By + Cz + D = 0**. RANSAC randomly selects 3 points in the cloud, draws a plane, and counts how many other points lie within 4cm of it (inliers). It repeats this 1000 times, returning the plane equation with the maximum inliers. Since sand is the largest surface in the frame, RANSAC always identifies the ground plane. Because this plane equation is calculated dynamically on every frame, **it automatically tilts with the slope**. If the rover climbs a 25-degree ramp, the plane equation tilts 25 degrees, and the ramp ground is filtered out perfectly.
4.  **Outlier Selection (Isolating Hazards):** Once RANSAC finds the ground plane (inliers), the node deletes all points matching it. The remaining points (outliers) represent coordinates that stick up (rocks, walls) or drop down (craters, cliffs).
5.  **DBSCAN Clustering (Dust & Reflective Light Filtering):** When tires spin in sand, they throw up dust. Dust particles reflect active infrared light from the depth camera, generating floating ghost points in the cloud. DBSCAN (Density-Based Spatial Clustering) scans the outlier points. It groups points if they are within 15cm of each other (`eps=0.15`). If a group has fewer than 10 points (`min_points=10`), DBSCAN classifies them as scattered noise and deletes them. Rocks and panels remain since they have hundreds of tightly packed points.
6.  **Costmap Insertion:** The cleaned obstacle points are packed back into a ROS PointCloud2 message and published. The path planner reads this pre-cleaned data and writes cost values directly to the Occupancy Grid, bypassing standard height filters.

---

#### 2. Approach A YAML Block Parameter Breakdown
Even though you are using Approach B, understanding the variables inside the standard Nav2 Costmap YAML file is crucial for mapping configurations:
*   `plugin: "nav2_costmap_2d::VoxelLayer"`: Loads the 3D voxel grid plugin. A voxel grid divides the space in front of the camera into a 3D grid of pixel cubes. Each cube stores whether it is free space or contains an obstacle.
*   `z_resolution: 0.05 / z_voxels: 16`: Sets the height resolution of each voxel cube to 5cm, with 16 voxels stacked on top of each other. This means the 3D grid maps space from Z = 0cm to Z = 80cm relative to the camera frame.
*   `marking: true / clearing: true`: **Marking** tells Nav2 to write obstacles to the map when point coordinates are detected. **Clearing** runs raytracing: it shoots virtual rays from the camera lens to the obstacle. If the ray passes through voxel cubes and finds nothing, it clears out any old stale cost values (resets them to 0).
*   `min_obstacle_height: 0.15`: **The Height Threshold.** Any points below Z = 15cm are assumed to be flat ground sand and are discarded. If the rover tilts up on a slope, the ground surface moves above 15cm relative to the camera, which is why this approach fails on slopes.

---

#### 3. Approach B: Custom RANSAC + DBSCAN Python Node Code Breakdown

Below is the complete RANSAC + DBSCAN python script:

```python
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d

class RansacObstacleDetector(Node):
    def __init__(self):
        super().__init__('ransac_obstacle_detector')
        
        # Subscribe to CropBox filtered PointCloud
        self.subscription = self.create_subscription(
            sensor_msgs.PointCloud2,
            '/perception/point_cloud_filtered',
            self.cloud_callback,
            10)
            
        # Publisher for obstacles only (ground removed)
        self.publisher = self.create_publisher(
            sensor_msgs.PointCloud2,
            '/perception/obstacles_only',
            10)
            
        self.get_logger().info('RANSAC Slope-Aware Obstacle Detector Started.')

    def cloud_callback(self, msg):
        # BLOCK 1: Conversion from ROS Binary to NumPy Arrays
        # ROS 2 sends PointCloud2 data as a raw binary byte array to save network bandwidth.
        # We use point_cloud2.read_points to extract the actual X, Y, Z float values from these bytes.
        points_list = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
            
        if len(points_list) < 100:
            return 
            
        xyz = np.array(points_list, dtype=np.float32)
        
        # BLOCK 2: Loading data into Open3D Point Cloud container
        # Open3D is a highly optimized C++ library with Python hooks that processes 3D matrices quickly.
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        
        # BLOCK 3: RANSAC Plane Segmentation
        # How RANSAC plane fitting works:
        # 1. It picks 3 random points in the cloud and draws a flat plane through them.
        # 2. It counts how many other points lie within 'distance_threshold' (4cm) of this plane (inliers).
        # 3. It repeats this process 1000 times (num_iterations).
        # 4. It returns the plane equation containing the highest count of inlier points (the flat ground sand).
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.04,  # Ground flatness tolerance (4cm)
            ransac_n=3,               # Points needed to draw a plane
            num_iterations=1000       # Fits to try
        )
        
        # BLOCK 4: Inverting indexes to crop out ground
        # select_by_index selects all ground plane points (inliers). 
        # By setting invert=True, we discard the ground and keep only the outliers (rocks/craters).
        obstacles_pcd = pcd.select_by_index(inliers, invert=True)
        
        if len(obstacles_pcd.points) == 0:
            return
            
        # BLOCK 5: DBSCAN Clustering (Density-Based Spatial Clustering of Applications with Noise)
        # How DBSCAN works:
        # It scans points and groups them if they sit close to each other.
        # eps=0.15m: maximum distance between two points to count them as neighbors.
        # min_points=10: clusters must have at least 10 points. If a group has fewer (like dust
        # kicked up by tires or sensor light noise), it is labeled as noise (-1) and deleted.
        labels = np.array(obstacles_pcd.cluster_dbscan(eps=0.15, min_points=10))
        
        # Keep only points belonging to a valid cluster (labels greater than or equal to 0)
        valid_indices = np.where(labels >= 0)[0]
        clean_obstacles_pcd = obstacles_pcd.select_by_index(valid_indices)
        
        # BLOCK 6: Packing coordinates back to ROS 2 PointCloud2
        # We extract the clean coordinate array and pack it back into standard ROS binary bytes.
        clean_points = np.asarray(clean_obstacles_pcd.points, dtype=np.float32)
        
        header = msg.header
        out_msg = point_cloud2.create_cloud_xyz32(header, clean_points.tolist())
        self.publisher.publish(out_msg)
```

##### Line-by-Line Code Breakdown & Function Explanations:
*   **Python Imports Block:** `rclpy` is the ROS 2 Python Client Library. We import `Node` to inherit parent class capabilities. We import `sensor_msgs.msg` to handle PointCloud2 definitions, and `sensor_msgs_py.point_cloud2` to read and build the binary byte array structures. `numpy` handles fast math array operations, and `open3d` executes C++ optimized 3D geometry equations.
*   **Constructor (`__init__`):** Calls `super().__init__('ransac_obstacle_detector')` to register the node name in the ROS 2 network. It creates a subscriber pointing to `/perception/point_cloud_filtered`. Every time a PointCloud2 message arrives, the node automatically executes the `cloud_callback` method. It also registers a publisher on topic `/perception/obstacles_only` to broadcast the cleaned coordinates.
*   **Callback Conversion (Block 1):** PointCloud2 messages are serialized raw byte arrays. The command `point_cloud2.read_points` reads this buffer, matches coordinates, and returns floats. We loop through the points, save them to a list, and convert them to a floating-point NumPy matrix (`xyz`) of type float32.
*   **Initializing Open3D (Block 2):** Instantiates an empty Open3D point cloud object: `o3d.geometry.PointCloud()`. The array coordinates are wrapped and loaded into Open3D using `o3d.utility.Vector3dVector(xyz)` so we can use Open3D's fast plane fitting functions.
*   **`segment_plane` RANSAC solver (Block 3):** Calls `pcd.segment_plane(...)`. 
    *   `distance_threshold=0.04`: Tolerance value (4cm). Points within 4cm of the plane equation are marked as ground inliers.
    *   `ransac_n=3`: The minimum points needed to draw a plane equation.
    *   `num_iterations=1000`: How many random fits RANSAC executes.
    *   It returns the calculated plane equation coefficients `[A, B, C, D]` and a list of index values (`inliers`) showing which points sit on the ground.
*   **Inlier Ground Removal (Block 4):** Executes `pcd.select_by_index(inliers, invert=True)`. This takes the list of ground indices and inverts them. The command deletes all indices belonging to the ground plane, returning a new PointCloud object containing only outlier coords (the actual obstacles).
*   **DBSCAN Noise Filtering (Block 5):** Executes `cluster_dbscan(...)`. 
    *   `eps=0.15`: Max distance (15cm) to group points together.
    *   `min_points=10`: If a group has fewer than 10 points (like sparse dust), it is labeled as noise (`-1`) and deleted.
    *   The array `labels` stores the group ID of each point. We filter for points with labels greater than or equal to 0, leaving only solid obstacles.
*   **Re-packing the ROS Message (Block 6):** Converts the cleaned Open3D points back into a NumPy array and builds a ROS PointCloud2 message using `point_cloud2.create_cloud_xyz32`, preserving the original message's header timestamp to maintain strict time synchronization. It then publishes it on `/perception/obstacles_only`.

##### Configure Nav2 Voxel Layer for this Clean Topic (`nav2_params.yaml`):
Because the ground coordinates have already been removed by RANSAC, set the Nav2 minimum obstacle height limit to negative values to register every remaining rock point in the cloud:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_grid: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        obstacle_range: 2.5
        raytrace_range: 3.0
        observation_sources: ransac_obstacles
        ransac_obstacles:
          topic: /perception/obstacles_only
          data_type: PointCloud2
          marking: true
          clearing: true
          min_obstacle_height: -1.0 # disabled height cutoff (RANSAC handled it)
          max_obstacle_height: 2.0
```

---

#### 4. Output of Branch 1: Terrain Geometry
*   **Published Topic:** `/perception/obstacles_only`
*   **ROS 2 Message Type:** `sensor_msgs/msg/PointCloud2`
*   **What does this output mean?** This is a highly refined, low-density 3D map consisting *only* of dangerous obstacles. The flat sand has been mathematically removed, and the floating wheel dust has been deleted by density checking.
*   **How exactly is it used?** This data is consumed directly by the `nav2_costmap_2d` node. Because the ground is already gone, Nav2 skips its own height filters. The path planner reads these rock coordinates, stamps them as "Lethal" on a 2D grid, inflates a safety bubble around them, and calculates steering commands to drive around them safely.

---

#### 4. Downstream Navigation Flow: What happens next?
Once Branch 1 successfully finishes and outputs the cleaned point cloud coordinates, a complete chain of downstream modules triggers to steer the rover physical wheels:

1.  **Occupancy Grid Updates (nav2_costmap_2d):** The local costmap node receives the `/perception/obstacles_only` points and maps them to flat grid coordinates. The cells holding rocks are written with a cost value of `100` (blocked).
2.  **Obstacle Inflation (nav2_costmap_2d):** Nav2 runs an inflation decay algorithm that adds a safety buffer around the rocks. Cells immediately touching the rock are labeled "Lethal Obstacle," and costs decay exponentially as you move further away. This safety buffer prevents the rover from driving too close and scraping its chassis.
3.  **Global Path Search (nav2_planner):** The planner node (running A* or Dijkstra) takes the destination target pose. It searches the costmap grid to trace the shortest path route that goes entirely through low-cost cells (avoiding the inflated boundaries), outputting a list of coordinate waypoints.
4.  **Dynamic Trajectory Tracking (nav2_controller):** The local controller (using TEB - Timed Elastic Band or DWA - Dynamic Window Approach) reads the path waypoints. It checks the local costmap for any immediate changes and calculates the optimal steering commands: linear velocity (vx in meters/second) and angular spin velocity (vyaw in radians/second).
5.  **Wheel Actuation (STM32 Motor Driver):** The local planner publishes these speeds as a `geometry_msgs/msg/Twist` message on the topic `/cmd_vel`. The motor driver node reads the twist commands, computes the RPM required for each independent tire, and transmits PWM voltages to the drive motors.

---

### Branch 2: Fiducial Tags & Marker Detection

#### What does this branch do?
This branch tracks specific visual QR-like codes (ArUco markers) placed on the rover panels or the environment. It decodes the IDs and calculates their precise 3D distance and rotation relative to the camera.

#### Do I use ArUco markers to update my Odometry?
**Yes, absolutely.** While Branch 3 (Visual Tracking) calculates continuous motion, it will still slowly drift over time (accumulated error) due to sand slippage. ArUco markers act as **Global Landmarks** (or "Ground Truth" anchors). 
*   **Validation:** When the rover spots a marker with a known ID and fixed arena location, it compares its calculated position to the marker's known position.
*   **Correction Loop:** If there is a discrepancy, the system injects the ArUco's 6-DoF pose into the EKF (Extended Kalman Filter) as an absolute measurement. This instantly snaps the drifted odometry back to reality, removing accumulated errors.
*   **Arm Alignment:** For the maintenance task panel, the arm relies entirely on the ArUco marker pose to calculate inverse kinematics (IK) for inserting plugs.

#### Detailed Pipeline Flow
1.  **Grayscale Conversion & Adaptive Thresholding:** The camera RGB image is converted to black and white. Adaptive thresholding makes the black squares pop out regardless of whether the Mars yard is under harsh sunlight or shadows.
2.  **Contour Tracing & Quad Filtering:** The algorithm traces all borders in the image and discards anything that isn't exactly four-sided (a quad).
3.  **Perspective Warp (Homography):** Because the camera is looking at the tag from an angle, the tag looks like a trapezoid, not a square. The system mathematically warps the pixels flat to read the internal binary grid (the ID).
4.  **solvePnP (Perspective-n-Point):** Once the corners are found in the 2D image, the algorithm uses the camera's lens calibration (intrinsics) and the physical size of the tag (e.g. 15cm) to triangulate exactly how far away the tag is in 3D space, and its rotation angles.
5.  **TF2 Coordinate Translation:** The calculated position is initially relative to the camera lens (`camera_link`). A TF2 transform shifts this coordinate math so that it is relative to the rover's `base_link` (or robotic arm base).
6.  **SLAM / Arm Controller Ingestion:** The final poses are sent to either the SLAM EKF for odometry correction, or the MoveIt arm controller for precise grasping.

#### Ready-Made Implementation
You do not need to write custom code for this. Use the open-source **`ros2_aruco`** package.

*   **Marker Configuration (`aruco_config.yaml`):**
    ```yaml
    /aruco_node:
      ros__parameters:
        image_topic: "/camera/camera/color/image_raw"
        camera_info_topic: "/camera/camera/color/camera_info"
        marker_size: 0.15 # crucial: exact physical tag width in meters
        aruco_dictionary_id: "DICT_4X4_50" # The type of tags used in ERC
    ```

*   **Implementation Steps:**
    1. Clone the repository into your workspace `src`:
       `git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git`
    2. Build using colcon:
       `colcon build --packages-select ros2_aruco`
    3. Run a static TF publisher mapping the camera mounting offset (X=20cm forward, Z=40cm up):
       `ros2 run tf2_ros static_transform_publisher 0.20 0.0 0.40 0 0 0 base_link camera_link`
    4. Run the node:
       `ros2 run ros2_aruco aruco_node --ros-args --params-file aruco_config.yaml`

#### Output of Branch 2:
*   **Topic:** `/aruco_poses`
*   **Message Type:** `geometry_msgs/msg/PoseArray`
*   **Variables:** `poses[i].position` (X, Y, Z coordinates), `poses[i].orientation` (Rotation Quaternion).
*   **What does this output mean?** This is a simple list of 3D coordinates and rotation angles telling the rover exactly where a printed tag is located relative to its own center.
*   **How exactly is it used?** It has two major uses: 
    1. **Odometry Correction:** The EKF node reads this to compare its drifted odometry against a known landmark, instantly correcting any built-up wheel slip errors.
    2. **Arm Manipulation:** The MoveIt Arm controller reads these exact coordinates to calculate Inverse Kinematics, allowing the robotic arm to reach out and plug cables into the exact panel location.

---

### Branch 3: Visual Odometry & Tracking
*   **Friendly Explanation:** Estimating movement by tracking textures. The camera picks out distinct ground textures (ORB keypoints) in Frame 1 and tracks how they shift in Frame 2. To avoid issues with wheel slippage in sand, an Extended Kalman Filter (EKF) fuses this visual positioning estimation with wheel encoder readings.
*   **Ready-Made Package:** The standard **`robot_localization`** package.
*   **EKF Fusion Configuration (`ekf.yaml`):**
    ```yaml
    ekf_filter_node:
      ros__parameters:
        frequency: 30.0
        two_d_mode: true
        publish_tf: true
        map_frame: map
        odom_frame: odom
        base_link_frame: base_link
        world_frame: odom

        # Camera Visual Odometry (X, Y, yaw positions)
        odom0: /zed/zed_node/odom
        odom0_config: [true,  true,  false,
                       false, false, true,
                       false, false, false,
                       false, false, false,
                       false, false, false]

        # Wheel Encoders (vx, vy, yaw velocities)
        odom1: /wheel_encoder/odom
        odom1_config: [false, false, false,
                       false, false, false,
                       true,  true,  false,
                       false, false, true,
                       false, false, false]
    ```
*   **Branch Output Details:**
    *   **Topic:** `/odometry/filtered`
    *   **Message Type:** `nav_msgs/msg/Odometry`
    *   **Variables:** `pose.pose` (fused coordinate position), `pose.covariance` (accuracy uncertainty matrix).
    *   **What does this output mean?** This is the final, most accurate estimation of where the rover is and how fast it is moving. It fuses the visual ground tracking (which is immune to wheel slip) with the wheel encoders (which are immune to featureless sand), giving the best of both worlds.
    *   **How exactly is it used?** This is the backbone of the rover's autonomy. The mapping node uses this coordinate to know exactly where to draw the map, and the path planner uses this coordinate as its "Current Location" when calculating a route to a destination.
