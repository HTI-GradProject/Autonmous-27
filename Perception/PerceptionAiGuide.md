# 🧠 Perception AI Implementation & Architecture Guide (`PerceptionAiGuide.md`)

> 💡 **AI Assistant Directive:** This document is the primary reference and roadmap for AI assistants and human engineers developing the Perception Subsystem. When completing a task or editing files, you **MUST update the task checkboxes (`- [x]`) and revision log** below.

---

## 📋 Metadata & Status Log
* **Subsystem:** Autonomous Perception Module
* **Target Compute:** NVIDIA Jetson Orin Nano (8GB CUDA GPU)
* **Primary Sensor:** Intel RealSense D435 RGB-D Camera
* **ROS 2 Middleware:** ROS 2 Humble / Jazzy
* **Target Workspace:** `/home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws`
* **Current Status:** Core Terrain Geometry algorithms implemented; Standardization, Persistent Memory, and ArUco 3D Vision pending.

---

## 📖 1. The Big Story: System Architecture & Context

### 1.1 High-Level Purpose
The **Perception Subsystem** provides environmental awareness for an autonomous Mars-analog rover competing in the European Rover Challenge (ERC). It converts raw sensor streams (3D point clouds and 2D camera images) into:
1. **3D Local Obstacle Bounding Boxes:** Extracted rocks and terrain barriers.
2. **Persistent Obstacle Memory:** Rock obstacles tracked and remembered across camera movements.
3. **ArUco Marker Telemetry:** 3D distance and 6-DOF pose estimation of mission target markers.

### 1.2 Division of Labor: Perception vs. Saif SLAM
To maximize performance on the Jetson Orin Nano, responsibilities between Perception and **Saif SLAM** (`saif SLAM.html`) are strictly divided:

```
+---------------------------------------------------------------------------------------------------+
|                                          HARDWARE LAYER                                           |
|                  Intel RealSense D435 RGB-D Camera + Wheel Encoders + BNO IMU                     |
+-------------------------------------------------+-------------------------------------------------+
                                                  |
                 +--------------------------------+--------------------------------+
                 | /camera/depth/color/points                                      | /camera/color/image_raw
                 v                                                                 v
+------------------------------------------------+               +-----------------------------------+
|            TERRAIN GEOMETRY NODE               |               |        ARUCO DETECTOR NODE        |
| 1. ROI Crop & base_link TF                     |               | 1. 2D Marker Corner Extraction    |
| 2. Ground Removal (Patchwork++ / RANSAC)       |               | 2. SolvePnP Math (Intrinsics)     |
| 3. Voxel Grid Downsampling (5cm leaf size)     |               | 3. 3D Distance & 6-DOF Pose       |
| 4. Radius Outlier Removal (ROR)                |               +-----------------+-----------------+
| 5. DBSCAN Euclidean Clustering                 |                                 |
+------------------------+-----------------------+                                 | /perception/aruco_pose
                         |                                                         v
                         | /perception/local_bboxes              +-----------------------------------+
                         v                                       |          SAIF SLAM MODULE         |
+------------------------------------------------+               | (RTAB-Map + EKF robot_localiz.)   |
|            PERSISTENT MEMORY NODE              |               | 1. EKF odom -> base_link (100 Hz) |
| 1. Spatial Nearest-Neighbor Matching           |               | 2. RTAB-Map map -> odom (1-5 Hz)  |
| 2. Exponential Moving Average (EMA Smoothing)  |               | 3. Hard reset on ArUco pose       |
| 3. Memory Retention (Blind Spot TTL Buffer)    |               +-----------------------------------+
+------------------------+-----------------------+
                         |
                         | /perception/obstacles_only
                         v
+---------------------------------------------------------------------------------------------------+
|                                        NAV2 COSTMAP SERVER                                        |
| Fuses persistent rock bounding boxes onto 2D occupancy costmap for Smac A* Path Planner           |
+---------------------------------------------------------------------------------------------------+
```

---

## 📂 2. Full Final Folder Structure (End-State Workspace)

Below is the complete, comprehensive directory structure of the Perception workspace when all development tasks are **100% finished**:

```
Autonmous_Ws/Perception/
├── PerceptionAiGuide.md                     # [THIS FILE] AI Roadmap & Master Reference
├── perception_Docu/
│   └── perception.html                      # Interactive Glassmorphism Architecture Report
├── ros dataset/                             # Recorded RealSense D435 bag files for testing
│   └── sample_rock_run.db3
│
└── terrain_geometry_improved/               # Core Perception ROS 2 Package Root
    ├── terrain_geometry_msgs/               # Custom ROS 2 Interface Package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── ObstacleFeature.msg          # Single obstacle telemetry (centroid, size, distance)
    │       └── ObstacleFeatureArray.msg     # Array of detected obstacle features
    │
    └── terrain_geometry/                    # Primary Python/C++ Perception Package
        ├── CMakeLists.txt
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        │
        ├── launch/                          # ROS 2 Launch Files
        │   ├── terrain.launch.py            # Standalone Terrain Geometry Launcher
        │   └── perception_system.launch.py  # Unified Perception System Launcher (All 3 Nodes)
        │
        ├── config/                          # Parameter Configuration Files
        │   ├── terrain_params.yaml          # Terrain Node Parameters (Ground, Voxel, DBSCAN)
        │   └── perception_params.yaml       # Central System Parameters (All Nodes & QoS)
        │
        ├── test/                            # Automated Unit & Integration Tests
        │   ├── test_ground_removal.py       # Unit test for RANSAC / Patchwork++ backends
        │   ├── test_clustering.py           # Unit test for DBSCAN cluster extraction
        │   ├── test_persistent_memory.py    # Integration test for EMA rock memory retention
        │   └── test_aruco_pnp.py            # Unit test for OpenCV SolvePnP 3D pose math
        │
        └── terrain_geometry/                # Python Source Code Package
            ├── __init__.py
            │
            ├── # --- EXECUTABLE ROS 2 NODES ---
            ├── terrain_node.py              # Node 1: Terrain Geometry & Clustering Executable
            ├── persistent_memory_node.py    # Node 2: Persistent Rock Memory Executable
            ├── aruco_detector_node.py       # Node 3: ArUco 3D Vision Executable
            │
            ├── # --- CORE ALGORITHMIC MODULES ---
            ├── roi_filter.py                # ROI Spatial Filter (base_link Crop Box)
            ├── ground_removal.py            # Ground Plane Removal (Patchwork++ / RANSAC)
            ├── voxel_filter.py              # 3D Voxel Grid Downsampling (5cm Leaf Size)
            ├── outlier_filter.py            # Radius Outlier Removal (ROR)
            ├── clustering.py                # DBSCAN Euclidean Clustering Engine
            ├── persistent_database.py       # EMA Rock Memory & Spatial Association Engine
            ├── aruco_pnp_solver.py          # OpenCV Corner Detector & SolvePnP Math
            ├── obstacle_features.py         # 3D Bounding Box & Telemetry Generator
            ├── obstacle_tracking.py         # Single-frame Nearest-Neighbor Associate
            ├── occupancy_grid.py            # 2D Grid Rasterizer
            ├── costmap_inflation.py         # Exponential Cost Inflation Buffer
            ├── tf_transform.py              # Coordinate Transform Utilities (tf2)
            ├── visualization.py             # RViz2 3D MarkerArray Publisher
            ├── benchmark.py                 # Algorithmic Benchmark Suite
            └── performance_profiling.py     # Real-time Execution Profiler
```

---

## 📊 3. Current Implementation Audit (What is DONE)

| Module / Component | Code File | Status | Description |
| :--- | :--- | :---: | :--- |
| **Main Terrain Node** | `terrain_node.py` | ✅ **DONE** | Orchestrates point cloud ingestion, processing pipeline, and publishing. |
| **Ground Removal** | `ground_removal.py` | ✅ **DONE** | Implements Patchwork++ and RANSAC ground plane segmentation. |
| **Voxel Downsampling** | `voxel_filter.py` | ✅ **DONE** | Reduces point cloud density by 90% using a 5cm voxel leaf size. |
| **Outlier Filter** | `outlier_filter.py` | ✅ **DONE** | Radius Outlier Removal (ROR) strips airborne noise and dust. |
| **DBSCAN Clustering** | `clustering.py` | ✅ **DONE** | Groups floating obstacle points into physical 3D rock clusters. |
| **Costmap Inflation** | `costmap_inflation.py` | ✅ **DONE** | Applies robot footprint radius inflation with exponential decay. |
| **Custom Messages** | `terrain_geometry_msgs` | ✅ **DONE** | Provides `ObstacleFeature.msg` and `ObstacleFeatureArray.msg`. |
| **Message Standardization** | `terrain_node.py` | 🔲 **TO-DO** | Update node to publish standard `vision_msgs/msg/Detection3DArray`. |
| **Persistent Memory Node** | `persistent_memory_node.py`| 🔲 **TO-DO** | Build database node using EMA smoothing for rock memory retention. |
| **ArUco Detector Node** | `aruco_detector_node.py` | 🔲 **TO-DO** | Build OpenCV SolvePnP node for 3D marker distance & pose estimation. |
| **Central Launcher** | `perception_system.launch.py`| 🔲 **TO-DO** | Unified launch script to spin up all 3 perception nodes concurrently. |

---

## 🎯 4. Sequential Task-by-Task Implementation Roadmap

Follow these small, modular tasks in sequential order. Check off (`- [x]`) each task as it is completed.

### Phase 1: Output Message Standardization
- [ ] **Task 01:** Add `vision_msgs` dependency to `package.xml` and `setup.py` inside `terrain_geometry`.
- [ ] **Task 02:** Update `obstacle_features.py` to add a converter method transforming `ObstacleFeatureArray` to `vision_msgs/msg/Detection3DArray`.
- [ ] **Task 03:** Modify `terrain_node.py` to publish raw local bounding boxes on `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).

### Phase 2: Persistent Memory Node Development
- [ ] **Task 04:** Create `persistent_database.py` implementing spatial Euclidean nearest-neighbor association logic.
- [ ] **Task 05:** Implement Exponential Moving Average (EMA) position smoothing equation in `persistent_database.py`:
  $$\mathbf{P}_{new} = \alpha \cdot \mathbf{P}_{detected} + (1 - \alpha) \cdot \mathbf{P}_{old}$$
- [ ] **Task 06:** Implement a Time-to-Live (TTL) memory buffer ($N = 5.0\text{s}$) to retain rocks when camera turns away.
- [ ] **Task 07:** Create `persistent_memory_node.py` subscribing to `/perception/local_bboxes`.
- [ ] **Task 08:** Publish persistent rock obstacles on `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and RViz markers on `/terrain/obstacle_markers`.

### Phase 3: ArUco 3D Vision Node Development
- [ ] **Task 09:** Create `aruco_pnp_solver.py` implementing OpenCV `cv2.aruco.detectMarkers()` corner extraction.
- [ ] **Task 10:** Implement `cv2.solvePnP()` in `aruco_pnp_solver.py` using camera intrinsic matrix $\mathbf{K}$ to compute 3D translation ($X, Y, Z$) and 6-DOF orientation.
- [ ] **Task 11:** Create `aruco_detector_node.py` subscribing to `/camera/color/image_raw` (`sensor_msgs/msg/Image`) and `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`).
- [ ] **Task 12:** Integrate `cv_bridge` to convert ROS image feeds into OpenCV BGR frames.
- [ ] **Task 13:** Publish target marker 3D pose on `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`).

### Phase 4: Centralized Parameters & System Launcher
- [ ] **Task 14:** Create `config/perception_params.yaml` consolidating parameters for all 3 perception nodes.
- [ ] **Task 15:** Configure ROS 2 QoS profiles (Best Effort for camera feeds, Reliable for output topics).
- [ ] **Task 16:** Create `launch/perception_system.launch.py` to launch `terrain_node`, `persistent_memory_node`, and `aruco_detector_node` together.

### Phase 5: Automated Testing & End-to-End Integration
- [ ] **Task 17:** Create unit tests `test/test_persistent_memory.py` and `test/test_aruco_pnp.py`.
- [ ] **Task 18:** Conduct Gazebo Mars Yard closed-loop navigation simulation test.
- [ ] **Task 19:** Verify Nav2 `global_costmap` subscribes to `/perception/obstacles_only` and paints cost zones.
- [ ] **Task 20:** Run `benchmark.py` on Jetson Orin Nano, confirming CPU usage remains $\le 40\%$.

---

## 🧪 5. Validation Protocols (Step-by-Step Testing Guidelines)

Execute these step-by-step verification commands after completing each group of tasks:

### 🧪 Validation Protocol 1: Message Standardization (After Tasks 01 – 03)
* **Step 1 (Build):**
  ```bash
  cd /home/saif/Desktop/MESEKET/Autonmous-27
  colcon build --packages-select terrain_geometry
  source install/setup.bash
  ```
* **Step 2 (Run Node):**
  ```bash
  ros2 launch terrain_geometry terrain.launch.py
  ```
* **Step 3 (Echo Topic):**
  ```bash
  ros2 topic echo /perception/local_bboxes vision_msgs/msg/Detection3DArray
  ```
* **Expect to see:** Terminal prints structured `vision_msgs/msg/Detection3DArray` messages containing 3D center coordinates ($X, Y, Z$) and dimensions for rocks in front of the rover.

---

### 🧪 Validation Protocol 2: Persistent Memory Node (After Tasks 04 – 08)
* **Step 1 (Run Memory Node):**
  ```bash
  ros2 run terrain_geometry terrain_node &
  ros2 run terrain_geometry persistent_memory_node
  ```
* **Step 2 (Verify Rate):**
  ```bash
  ros2 topic hz /perception/obstacles_only
  ```
* **Step 3 (Memory Retention Test):**
  Drive the rover towards a rock in Gazebo, then rotate the camera 90 degrees away into a blind spot.
* **Expect to see:** Topic `/perception/obstacles_only` continues publishing the rock's smoothed coordinates for 5+ seconds after it leaves the camera's field-of-view.

---

### 🧪 Validation Protocol 3: ArUco 3D Vision Node (After Tasks 09 – 13)
* **Step 1 (Launch Gazebo World):**
  ```bash
  ros2 launch worlds world_Rotated_Aruco.launch.py
  ```
* **Step 2 (Run ArUco Node):**
  ```bash
  ros2 run terrain_geometry aruco_detector_node
  ```
* **Step 3 (Echo Marker Pose):**
  ```bash
  ros2 topic echo /perception/aruco_pose geometry_msgs/msg/PoseStamped
  ```
* **Expect to see:** Terminal outputs a `PoseStamped` message detailing the exact 3D distance ($X$ meters ahead, $Y$ offset) and orientation matching the physical ArUco marker in Gazebo.

---

### 🧪 Validation Protocol 4: System Launcher & Parameters (After Tasks 14 – 16)
* **Step 1 (Launch System):**
  ```bash
  ros2 launch terrain_geometry perception_system.launch.py
  ```
* **Step 2 (Check Active Nodes):**
  ```bash
  ros2 node list
  ```
* **Expect to see:** `/terrain_node`, `/persistent_memory_node`, and `/aruco_detector_node` are all active simultaneously with zero QoS mismatch warnings in `ros2 doctor`.

---

### 🧪 Validation Protocol 5: Full End-to-End System Test (After Tasks 17 – 20)
* **Step 1 (Launch Full Stack):**
  ```bash
  ros2 launch my_robot_description gazebo.launch.py world:=world1.world
  ros2 launch terrain_geometry perception_system.launch.py
  ```
* **Step 2 (Inspect RViz):**
  ```bash
  rviz2
  ```
  Add display `/global_costmap/costmap` (`OccupancyGrid`) and `/terrain/obstacle_markers` (`MarkerArray`).
* **Expect to see:** Rocks in Gazebo appear as solid red cost regions on the RViz map. The rover plans autonomous driving paths around rocks without collision while keeping Jetson CPU usage under 40%.
