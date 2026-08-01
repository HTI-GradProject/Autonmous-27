# Perception Module: Terrain Geometry & Obstacle Detection

This module processes raw 3D depth camera feeds into local costmaps and obstacle arrays, enabling the path planner to safely route the rover around rocks, slopes, and crater terrain.

---

## 1. Input & Output Topics

The core processing node (`terrain_node`) communicates over the following ROS 2 interfaces:

### Subscriptions (Input)
*   **`/camera/depth/color/points`** (`sensor_msgs/msg/PointCloud2`)
    *   Raw 3D point cloud in the camera optical frame.
*   **`/tf` & `/tf_static`** (`tf2_msgs/msg/TFMessage`)
    *   D435i camera sensor to base coordinate frame transform (`camera_link` $\rightarrow$ `base_link`).

### Publications (Output)
*   **`/terrain/costmap`** (`nav_msgs/OccupancyGrid`)
    *   Unified inflated obstacle costmap ready for SLAM or path planners.
*   **`/terrain/obstacle_features`** (`terrain_geometry_msgs/ObstacleFeatureArray`)
    *   Geometric telemetry for detected obstacles (centroid, size dimensions, distance).
*   **`/terrain/obstacle_markers`** (`visualization_msgs/MarkerArray`)
    *   3D visual bounding boxes and text labels for RViz2 display.

### Debug Publications (Active if `publish_debug_topics` is `True`)
*   **`/terrain/debug/ground_cloud`** (`sensor_msgs/PointCloud2`): Ground planes separated by Ground Removal.
*   **`/terrain/debug/voxel_cloud`** (`sensor_msgs/PointCloud2`): Point cloud filtered by Voxel Downsampling.
*   **`/terrain/debug/clustered_cloud`** (`sensor_msgs/PointCloud2`): Points colored by DBSCAN Cluster IDs.

---

## 2. Running & Using the Terrain Geometry Node

Make sure your workspace is built and sourced:
```bash
# Compile and source workspace from the repository root (outside Autonmous_Ws)
cd /path/to/your/cloned/repository
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To run the Terrain Geometry node standalone with default configurations:
```bash
ros2 launch terrain_geometry terrain.launch.py
```

---

## 3. Testing Methods & Validation Guide

You can validate the perception node using two distinct workflows:

### Method A: Testing with the Simulated Rover (Recommended)
This closed-loop test runs the active rover simulation inside the Mars Yard world in Gazebo alongside the perception node.

1.  **Start the World & Rover Simulation:**
    Choose one of the following two options:
    *   **Option 1 (Two Commands):**
        First launch the world:
        ```bash
        ros2 launch worlds world1.launch.py
        ```
        Then, in a separate terminal, spawn the rover:
        ```bash
        ros2 launch my_robot_description spawn_rover.launch.py
        ```
    *   **Option 2 (Single Command):**
        Launch the world and spawn the rover together:
        ```bash
        ros2 launch my_robot_description gazebo.launch.py world:=world1.world
        ```
2.  **Launch the Terrain Geometry Node:**
    ```bash
    ros2 launch terrain_geometry terrain.launch.py
    ```
3.  **Drive the Rover:**
    Open a terminal to teleoperate the rover:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
4.  **Visualize results in RViz2:**
    Run RViz2:
    ```bash
    rviz2
    ```
    *   Add `/terrain/costmap` (OccupancyGrid display) to see real-time obstacle avoidance layers updating as you approach rocks.
    *   Add `/terrain/obstacle_markers` (MarkerArray display) to see 3D bounding boxes drawn around detected stones.

---

### Method B: Testing with a ROS Bag (RealSense Log Playback)
This offline validation runs the perception pipeline using pre-recorded depth camera bag files.

1.  **Publish a Static Transform (If needed):**
    Because the recorded bag might lack TF transforms mapping the camera to the rover frame, broadcast a static transform so the TF transformer doesn't fail:
    ```bash
    ros2 run tf2_ros static_transform_publisher 0.5 0 0.5 0 0 0 base_link camera_link
    ```
2.  **Play the ROS Bag:**
    Run the playback command on your local bag file:
    ```bash
    # For ROS 2 (.db3 or metadata.yaml bag formats):
    ros2 bag play /path/to/your/bag_file/
    ```
    > [!TIP]
    > If the bag records the point cloud on a different topic than `/camera/depth/color/points`, remap it:
    > ```bash
    > ros2 bag play /path/to/your/bag_file/ --remap /original_pointcloud_topic:=/camera/depth/color/points
    > ```
3.  **Launch the Terrain Geometry Node:**
    ```bash
    ros2 launch terrain_geometry terrain.launch.py
    ```
4.  **Verify the Outputs:**
    Open RViz2 to verify `/terrain/costmap` and `/terrain/obstacle_markers` output matching the playback stream.

---

### What Data You Can Visualize and Validate in RViz

You can visualize all data from the camera and the perception node. Here is the checklist of what to add (using the **Add** button in the bottom-left of RViz):

| Data to Validate | RViz Display Type | Topic | What to look for |
| :--- | :--- | :--- | :--- |
| **Raw Camera Video** | Image | `/camera/image_raw` | Live RGB video feed from the front of the rover. |
| **Raw 3D Point Cloud** | PointCloud2 | `/camera/depth/color/points` | 3D colored points showing the terrain directly in front of the rover. |
| **3D Obstacle Boxes** | MarkerArray | `/terrain/obstacle_markers` | 3D bounding boxes drawn around rocks, labeled with their cluster ID. |
| **Perception Costmap** | OccupancyGrid | `/terrain/costmap` | A flat 2D grid overlay showing safe areas (clear) and dangerous areas (occupied by rocks in red/purple). |
| **Debug Ground Cloud** | PointCloud2 | `/terrain/debug/ground_cloud` | Ground points separated by Patchwork++ (colored in solid green/gray). |
| **Debug Clusters** | PointCloud2 | `/terrain/debug/clustered_cloud` | Remaining points after ground removal, colored by DBSCAN cluster ID. |

> [!IMPORTANT]
> **QoS Configuration (Avoiding Warnings & Missing Data):**
> If you see warnings or if raw feeds (video, point clouds) do not display, check the **Reliability Policy** in the settings of the RViz display element:
> * Change it from **Reliable** to **Best Effort** for `/camera/image_raw` and `/camera/depth/color/points`.
> * RealSense cameras and Gazebo simulations publish high-bandwidth sensor topics using a *Best Effort* QoS profile. RViz will output status warnings and fail to show data if its profile is mismatched (left on *Reliable*). Changing them to *Best Effort* immediately resolves the warnings and displays the streams.
