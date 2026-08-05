# 🧠 Perception AI Implementation Guide & GitHub Issue Breakdown (`PerceptionAiGuide.md`)

> 💡 **AI Maintenance Directive:** This document serves as the master roadmap for AI assistants and engineers implementing the Perception Subsystem. When completing a task or GitHub Issue, **mark the corresponding task checkboxes (`- [x]`) and update the Document Status Log.**

---

## 📋 Metadata & Document Control
* **Subsystem:** Autonomous Perception Module
* **Target Hardware:** NVIDIA Jetson Orin Nano (8GB CUDA GPU) + Intel RealSense D435 RGB-D Camera
* **ROS 2 Environment:** ROS 2 Humble / Jazzy
* **Workspace Path:** `/home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Perception`
* **GitHub Project Label:** `label:Perception`
* **Current Status:** Core Terrain Geometry algorithms (`terrain_node.py`) implemented; Standardization, Persistent Memory, and ArUco 3D Vision Issues pending.

---

## 📖 1. The Big Picture & System Context

The **Perception Subsystem** provides 3D spatial awareness and visual target recognition for an autonomous Mars-analog rover. To optimize compute performance on the Jetson Orin Nano, the architecture strictly separates **State Estimation/SLAM** (handled by the **Saif SLAM Module**) from **Obstacle Recognition & Target Pose Estimation** (handled by this **Perception Module**).

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

## 📂 2. Full End-State Folder Structure

```
Autonmous_Ws/Perception/
├── PerceptionAiGuide.md                     # [THIS FILE] AI Roadmap & Master Reference
├── perception_Docu/
│   └── perception.html                      # Interactive Glassmorphism Architecture Report
├── ros dataset/                             # Recorded RealSense D435 bag files for testing
│
└── terrain_geometry_improved/               # Core Perception ROS 2 Package Root
    ├── terrain_geometry_msgs/               # Custom ROS 2 Interface Package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── ObstacleFeature.msg          # Single obstacle telemetry (centroid, size, distance)
    │       └── ObstacleFeatureArray.msg     # Array of detected obstacle features
    │
    └── terrain_geometry/                    # Primary Perception Package
        ├── CMakeLists.txt / setup.py
        ├── package.xml
        ├── launch/
        │   ├── terrain.launch.py            # Standalone Terrain Geometry Launcher
        │   └── perception_system.launch.py  # Unified System Launcher (All 3 Nodes)
        ├── config/
        │   ├── terrain_params.yaml          # Terrain Node Parameters (Ground, Voxel, DBSCAN)
        │   └── perception_params.yaml       # Central System Parameters & QoS Profiles
        ├── test/
        │   ├── test_ground_removal.py       # Unit test for RANSAC / Patchwork++ backends
        │   ├── test_clustering.py           # Unit test for DBSCAN cluster extraction
        │   ├── test_persistent_memory.py    # Integration test for EMA rock memory retention
        │   └── test_aruco_pnp.py            # Unit test for OpenCV SolvePnP 3D pose math
        └── terrain_geometry/                # Python Source Code
            ├── __init__.py
            ├── terrain_node.py              # Node 1: Terrain Geometry & Clustering Executable
            ├── persistent_memory_node.py    # Node 2: Persistent Rock Memory Executable
            ├── aruco_detector_node.py       # Node 3: ArUco 3D Vision Executable
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

## 📊 3. Implementation Audit: What is DONE vs. TO-DO

| Module / Component | Code File | Status | Description |
| :--- | :--- | :---: | :--- |
| **Main Terrain Node** | `terrain_node.py` | ✅ **DONE** | Orchestrates point cloud ingestion, processing pipeline, and publishing. |
| **Ground Removal** | `ground_removal.py` | ✅ **DONE** | Implements Patchwork++ and RANSAC ground plane segmentation. |
| **Voxel Downsampling** | `voxel_filter.py` | ✅ **DONE** | Reduces point cloud density by 90% using a 5cm voxel leaf size. |
| **Outlier Filter** | `outlier_filter.py` | ✅ **DONE** | Radius Outlier Removal (ROR) strips airborne noise and dust. |
| **DBSCAN Clustering** | `clustering.py` | ✅ **DONE** | Groups floating obstacle points into physical 3D rock clusters. |
| **Costmap Inflation** | `costmap_inflation.py` | ✅ **DONE** | Applies robot footprint radius inflation with exponential decay. |
| **Custom Messages** | `terrain_geometry_msgs` | ✅ **DONE** | Provides `ObstacleFeature.msg` and `ObstacleFeatureArray.msg`. |
| **Standardized Bounding Boxes** | `terrain_node.py` | 🔲 **Task 1A Series** | Update node to publish standard `vision_msgs/msg/Detection3DArray`. |
| **Persistent Memory Node** | `persistent_memory_node.py`| 🔲 **Task 2A Series** | Build database node using EMA smoothing for rock memory retention. |
| **ArUco Detector Node** | `aruco_detector_node.py` | 🔲 **Task 3A Series** | Build OpenCV SolvePnP node for 3D marker distance & pose estimation. |
| **Central System Launcher** | `perception_system.launch.py`| 🔲 **Task 4A Series** | Unified launch script to spin up all 3 perception nodes concurrently. |
| **System Validation & Test** | `test/` | 🔲 **Task 5A Series** | Automated unit tests and end-to-end simulation validation. |

---

## 🎫 4. GitHub Issue & Implementation Task Guide

Below is the complete list of GitHub Issues and Task breakdowns formatted for direct entry into GitHub Project boards.

---

### 📦 ISSUE 1: `[Perception] [Task 1A.1 & 1A.2] Standardize Interface Messages`
* **GitHub Issue Title:** `[Perception] [Task 1A.1 & 1A.2] Standardize Interface Messages to vision_msgs`
* **GitHub Labels:** `label:Perception`, `label:enhancement`
* **Sub-Tasks:**
  - [ ] `- [Task 1A.1]` Add `vision_msgs` dependency to `package.xml` and `setup.py` inside `terrain_geometry`.
  - [ ] `- [Task 1A.2]` Implement converter in `obstacle_features.py` transforming `ObstacleFeatureArray` to `vision_msgs/msg/Detection3DArray`.
  - [ ] `- [Task 1A.3]` Modify `terrain_node.py` to publish `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).

#### 🛠️ Implementation Overview
Update the `terrain_geometry` package dependencies to include ROS 2 standard `vision_msgs`. Add a converter utility method `to_detection3d_array()` in `obstacle_features.py` that maps bounding box centroids, orientation, and sizes ($dx, dy, dz$) into standard `vision_msgs/msg/Detection3D` objects. Update `terrain_node.py` to construct a publisher on topic `/perception/local_bboxes`.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Build workspace
cd /home/saif/Desktop/MESEKET/Autonmous-27
colcon build --packages-select terrain_geometry
source install/setup.bash

# Step 2: Run Terrain Node
ros2 launch terrain_geometry terrain.launch.py

# Step 3: Echo standardized bounding box topic
ros2 topic echo /perception/local_bboxes vision_msgs/msg/Detection3DArray
```
* **Expect to see:** Terminal prints valid `vision_msgs/msg/Detection3DArray` messages containing 3D bounding box center poses and dimensions for detected obstacles in front of the rover.

---

### 🎯 CHECKPOINT TASK 1: `[Perception Checkpoint 1] Validate Raw Bounding Box Interface`
* **GitHub Issue Title:** `[Perception Checkpoint 1] Validate Raw Local Bounding Box Publishing`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Description:** Milestone verification gate ensuring `/perception/local_bboxes` streams clean `vision_msgs/msg/Detection3DArray` messages with correct coordinate frames (`base_link`) without dropping frames or leaking memory.

#### 🧪 Checkpoint Validation Steps
- [ ] Run `terrain_node` in Gazebo Mars Yard simulation.
- [ ] Verify message frequency using `ros2 topic hz /perception/local_bboxes` (Target: $\ge 10\text{ Hz}$).
- [ ] Confirm frame ID in header matches `base_link`.
* **Expect to see:** Reliable $10+\text{ Hz}$ stream of 3D bounding boxes corresponding accurately to physical Gazebo rocks.

---

### 📦 ISSUE 2: `[Perception] [Task 2A.1 & 2A.2] Spatial Matching & EMA Smoothing Engine`
* **GitHub Issue Title:** `[Perception] [Task 2A.1 & 2A.2] Implement Spatial Association & EMA Rock Position Smoothing`
* **GitHub Labels:** `label:Perception`, `label:feature`
* **Sub-Tasks:**
  - [ ] `- [Task 2A.1]` Create `persistent_database.py` implementing 3D spatial Euclidean nearest-neighbor data association.
  - [ ] `- [Task 2A.2]` Implement Exponential Moving Average (EMA) position smoothing equation:
    $$\mathbf{P}_{new} = \alpha \cdot \mathbf{P}_{detected} + (1 - \alpha) \cdot \mathbf{P}_{old}$$
  - [ ] `- [Task 2A.3]` Add tracking ID assignment to prevent duplicate rock entries.

#### 🛠️ Implementation Overview
Create `persistent_database.py` containing a `PersistentDatabase` class. When a new array of 3D bounding boxes (`/perception/local_bboxes`) arrives, perform a Euclidean distance threshold check ($d_{match} \le 0.5\text{m}$) against currently stored rock tracks. If a match is found, update its position using EMA smoothing ($\alpha = 0.4$) to eliminate camera jitter. If no match is found, register a new rock track.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Run unit test script for persistent database
colcon build --packages-select terrain_geometry
python3 -m unittest terrain_geometry.test.test_persistent_memory
```
* **Expect to see:** All unit tests pass, confirming noisy detection inputs settle smoothly onto true rock coordinates without spawning duplicate tracks.

---

### 📦 ISSUE 3: `[Perception] [Task 2A.3 & 2A.4] Persistent Memory Node & Blind Spot Retention`
* **GitHub Issue Title:** `[Perception] [Task 2A.3 & 2A.4] Implement Persistent Memory Node with TTL Retention`
* **GitHub Labels:** `label:Perception`, `label:feature`
* **Sub-Tasks:**
  - [ ] `- [Task 2A.4]` Implement Time-to-Live (TTL) memory buffer ($N = 5.0\text{s}$) in `persistent_database.py` to retain rocks in camera blind spots.
  - [ ] `- [Task 2A.5]` Create `persistent_memory_node.py` subscribing to `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).
  - [ ] `- [Task 2A.6]` Publish persistent rock obstacles on `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and RViz markers on `/terrain/obstacle_markers`.

#### 🛠️ Implementation Overview
Construct `persistent_memory_node.py` as a standalone ROS 2 executable node. Instantiate `PersistentDatabase` and subscribe to `/perception/local_bboxes`. Add a timer loop ($10\text{ Hz}$) that decrements track TTL counters for unobserved rocks and purges tracks exceeding $5.0\text{s}$. Publish the active persistent rock array on `/perception/obstacles_only` and publish 3D bounding box visual markers on `/terrain/obstacle_markers`.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Launch terrain node and persistent memory node
ros2 run terrain_geometry terrain_node &
ros2 run terrain_geometry persistent_memory_node

# Step 2: Test memory retention in Gazebo
# Action: Drive rover towards a rock, then rotate camera 90 degrees away into a blind spot.
ros2 topic echo /perception/obstacles_only
```
* **Expect to see:** Topic `/perception/obstacles_only` continues publishing the rock's smoothed coordinates for 5 seconds after it leaves the camera's field of view.

---

### 🎯 CHECKPOINT TASK 2: `[Perception Checkpoint 2] Persistent Rock Memory & TTL Retention Verification`
* **GitHub Issue Title:** `[Perception Checkpoint 2] Validate Memory Retention & Spatial Association`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Description:** Milestone verification gate confirming the Persistent Memory Node smooths camera jitter, prevents duplicate rock entries, and retains blind-spot obstacles for Nav2 costmap overlay.

#### 🧪 Checkpoint Validation Steps
- [ ] Verify topic publishing rate: `ros2 topic hz /perception/obstacles_only` (Target: $10\text{ Hz}$).
- [ ] Confirm RViz display displays persistent bounding boxes on `/terrain/obstacle_markers` even when camera turns away.
* **Expect to see:** Rock markers remain static and visible on RViz map display when camera rotates away.

---

### 📦 ISSUE 4: `[Perception] [Task 3A.1 & 3A.2] OpenCV ArUco Corner Extraction & SolvePnP Math`
* **GitHub Issue Title:** `[Perception] [Task 3A.1 & 3A.2] Implement OpenCV ArUco Detection & 3D SolvePnP Solver`
* **GitHub Labels:** `label:Perception`, `label:feature`
* **Sub-Tasks:**
  - [ ] `- [Task 3A.1]` Create `aruco_pnp_solver.py` implementing `cv2.aruco.detectMarkers()` corner extraction.
  - [ ] `- [Task 3A.2]` Implement `cv2.solvePnP()` in `aruco_pnp_solver.py` using camera intrinsics $\mathbf{K}$ to compute 3D translation ($X, Y, Z$) and 6-DOF orientation.
  - [ ] `- [Task 3A.3]` Add distance clipping ($d_{max} = 4.0\text{m}$) and subpixel corner refinement.

#### 🛠️ Implementation Overview
Create `aruco_pnp_solver.py` containing an `ArUcoPnPSolver` class. Given a 2D BGR image and camera calibration parameters ($f_x, f_y, c_x, c_y$), detect ArUco dictionary markers (`DICT_5X5_250`). Pass the 4 2D corner pixels and known physical marker size ($0.2\text{m}$) to `cv2.solvePnP()` using `SOLVEPNP_IPPE_SQUARE`. Convert the output rotation vector ($\mathbf{rvec}$) and translation vector ($\mathbf{tvec}$) into 3D meters ($X, Y, Z$) and quaternion orientation.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Run unit test for SolvePnP math
colcon build --packages-select terrain_geometry
python3 -m unittest terrain_geometry.test.test_aruco_pnp
```
* **Expect to see:** Unit test passes, verifying calculated 3D distance matches ground truth pixel projection to within $< 1\text{cm}$ error.

---

### 📦 ISSUE 5: `[Perception] [Task 3A.3 & 3A.4] ArUco Vision Node & Pose Publisher`
* **GitHub Issue Title:** `[Perception] [Task 3A.3 & 3A.4] Implement ArUco Detector Node & Target Pose Publisher`
* **GitHub Labels:** `label:Perception`, `label:feature`
* **Sub-Tasks:**
  - [ ] `- [Task 3A.4]` Create `aruco_detector_node.py` subscribing to `/camera/color/image_raw` (`sensor_msgs/msg/Image`) and `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`).
  - [ ] `- [Task 3A.5]` Integrate `cv_bridge` to convert ROS images into OpenCV BGR format.
  - [ ] `- [Task 3A.6]` Publish target pose on `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`).
  - [ ] `- [Task 3A.7]` Add option to broadcast static TF transform `camera_link -> aruco_marker_ID`.

#### 🛠️ Implementation Overview
Construct `aruco_detector_node.py` as a ROS 2 executable node. Instantiate `ArUcoPnPSolver`. Use `cv_bridge` to convert incoming `/camera/color/image_raw` frames. Extract camera intrinsics from `/camera/camera_info`. Solve 3D marker pose and publish standard `geometry_msgs/msg/PoseStamped` messages on `/perception/aruco_pose` for Saif SLAM loop closures and mission state machine navigation.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Launch Gazebo world with ArUco marker
ros2 launch worlds world_Rotated_Aruco.launch.py &

# Step 2: Run ArUco Detector Node
ros2 run terrain_geometry aruco_detector_node

# Step 3: Echo output pose
ros2 topic echo /perception/aruco_pose geometry_msgs/msg/PoseStamped
```
* **Expect to see:** Terminal outputs a valid `PoseStamped` message detailing the exact 3D distance ($X$ meters ahead, $Y$ offset, $Z$ height) matching the Gazebo ArUco marker location.

---

### 🎯 CHECKPOINT TASK 3: `[Perception Checkpoint 3] ArUco 3D Pose Estimation Verification`
* **GitHub Issue Title:** `[Perception Checkpoint 3] Validate ArUco 3D Distance & Pose Accuracy`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Description:** Milestone verification gate confirming the ArUco vision node accurately estimates 3D target distance and pose without relying on depth sensor IR noise.

#### 🧪 Checkpoint Validation Steps
- [ ] Confirm `/perception/aruco_pose` updates reliably when marker enters camera view.
- [ ] Verify 3D translation ($X, Y, Z$) error is $\le 2\text{cm}$ at $2.0\text{m}$ distance in simulation.
* **Expect to see:** Clean pose updates received by Saif SLAM without coordinate jumps or missing headers.

---

### 📦 ISSUE 6: `[Perception] [Task 4A.1, 4A.2 & 4A.3] System Launch & Parameter Centralization`
* **GitHub Issue Title:** `[Perception] [Task 4A.1, 4A.2 & 4A.3] Create Centralized YAML Parameters & System Launcher`
* **GitHub Labels:** `label:Perception`, `label:enhancement`
* **Sub-Tasks:**
  - [ ] `- [Task 4A.1]` Create `config/perception_params.yaml` consolidating parameters for all 3 perception nodes.
  - [ ] `- [Task 4A.2]` Configure ROS 2 QoS profiles (Best Effort for camera inputs, Reliable for output topics).
  - [ ] `- [Task 4A.3]` Create `launch/perception_system.launch.py` to spin up `terrain_node`, `persistent_memory_node`, and `aruco_detector_node` concurrently.

#### 🛠️ Implementation Overview
Create `config/perception_params.yaml` defining parameters for all three nodes (`terrain_node`, `persistent_memory_node`, `aruco_detector_node`), including ROI bounds, voxel leaf size, DBSCAN epsilon, EMA alpha, and QoS settings. Create `launch/perception_system.launch.py` using ROS 2 `LaunchDescription` and `Node` actions to launch all three nodes seamlessly with standard parameter loading.

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Build and launch entire system
colcon build --packages-select terrain_geometry
source install/setup.bash
ros2 launch terrain_geometry perception_system.launch.py

# Step 2: Check running nodes
ros2 node list
```
* **Expect to see:** Nodes `/terrain_node`, `/persistent_memory_node`, and `/aruco_detector_node` are all running concurrently without parameter parsing errors or QoS mismatch warnings.

---

### 🎯 CHECKPOINT TASK 4: `[Perception Checkpoint 4] Multi-Node System Launch & QoS Validation`
* **GitHub Issue Title:** `[Perception Checkpoint 4] Validate System Launcher & QoS Compatibility`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Description:** Milestone verification gate confirming all three nodes execute concurrently from a single launch file without camera QoS mismatch warnings or memory leaks.

#### 🧪 Checkpoint Validation Steps
- [ ] Run `ros2 doctor --report` and verify zero QoS mismatch warnings on camera topics.
- [ ] Monitor CPU/RAM overhead using `htop` (Target: total CPU usage $\le 40\%$).
* **Expect to see:** All 3 nodes running stably at $10-15\text{ Hz}$ on the Jetson compute stack.

---

### 📦 ISSUE 7: `[Perception] [Task 5A.1 & 5A.2] End-to-End Nav2 Integration & Benchmarking`
* **GitHub Issue Title:** `[Perception] [Task 5A.1 & 5A.2] Validate End-to-End Nav2 Integration & Performance Benchmarking`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Sub-Tasks:**
  - [ ] `- [Task 5A.1]` Conduct closed-loop simulation test in Gazebo Mars Yard world (`world1.world`).
  - [ ] `- [Task 5A.2]` Verify Nav2 `global_costmap` subscribes to `/perception/obstacles_only` and inflates rock cost zones.
  - [ ] `- [Task 5A.3]` Execute `benchmark.py` and log algorithmic execution timing in `perception_Docu/`.

#### 🛠️ Implementation Overview
Perform closed-loop testing with the full autonomous stack (Gazebo simulation + Nav2 + Saif SLAM + Perception). Verify that persistent 3D bounding box obstacles published on `/perception/obstacles_only` are ingested by Nav2's `costmap_2d` server, creating inflated cost regions on the global map. Execute `benchmark.py` to record per-stage execution latency (Ground Removal, Voxel Grid, DBSCAN) and confirm real-time performance ($\le 60\text{ms}$ total pipeline latency per frame).

#### 🧪 Testing & Validation Guide
```bash
# Step 1: Launch full rover simulation + Nav2 + Perception
ros2 launch my_robot_description gazebo.launch.py world:=world1.world &
ros2 launch terrain_geometry perception_system.launch.py &

# Step 2: Open RViz2 and display costmap
rviz2
```
* **Expect to see:** Rocks in Gazebo appear as solid red inflated cost zones on the RViz costmap display. The rover navigates autonomously around rocks without collision while keeping total perception pipeline latency $\le 60\text{ms}$.

---

### 🏆 CHECKPOINT TASK 5: `[Perception Final Gateway] Full Subsystem Verification & Competition Readiness`
* **GitHub Issue Title:** `[Perception Final Gateway] Validate Full Subsystem Integration & Performance Metrics`
* **GitHub Labels:** `label:Perception`, `label:QA/Testing`
* **Description:** Final gateway sign-off verifying that all perception nodes, topics, memory retention, ArUco 3D vision, and Nav2 costmap overlays operate flawlessly.

#### 🧪 Checkpoint Validation Steps
- [ ] Verify zero collisions during 15-minute autonomous navigation run in Gazebo Mars Yard.
- [ ] Confirm ArUco target markers trigger landmark reset events in Saif SLAM.
- [ ] Verify execution benchmark report saved to `perception_Docu/benchmark_report.txt`.
* **Expect to see:** Complete autonomous perception pipeline operates cleanly, passing all ERC competition navigation benchmarks.

---

## 🛠️ 5. AI Developer Maintenance Instructions
When implementing or editing code in this repository:
1. Open this file (`PerceptionAiGuide.md`).
2. Mark completed sub-tasks using `- [x]`.
3. Update Section 3 (**Implementation Audit**) table.
4. Keep all issue titles and test commands aligned with the workspace.
