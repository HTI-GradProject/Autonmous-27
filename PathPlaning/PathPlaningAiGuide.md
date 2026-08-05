# Path Planning Module: AI Implementation Guide

This document serves as the master guide and checklist for AI agents working on the Path Planning module for the ERC 2026 Rover. It includes the architectural context, the target folder structure, and a highly detailed breakdown of implementation tasks, specifically designed for **two developers working in parallel**.

*Note to AI: Update this document (using file editing tools) whenever a task is completed or the folder structure changes.*

---

## 1. The Big Story: Architecture
This implementation covers the full integration of both the **Smac Hybrid A* Global Planner** and the **MPPI Local Planner & Controller** via ROS 2 Nav2 plugins (`nav2_smac_planner` and `nav2_mppi_controller`).

**System Data Flow:**
*   **Inputs:** `/goal_pose` (Target), `/map` (Global static map), `/perception/obstacles` (Real-time dynamic obstacles), `/odometry/filtered` (High-speed 100 Hz EKF odometry), and TF transforms (`map -> odom -> base_link`).
*   **Outputs:** `/plan` (The route) and `/cmd_vel` (Motor commands).

---

## 2. Target Folder Structure
This is the directory structure that has been created for the implementation phase. 

```text
PathPlaning/
├── README.md
├── PathPlaningAiGuide.md
├── PathPlanner_Docu/
│   └── PathPlannerGuide.html
└── src/
    └── erc_path_planner/
        ├── package.xml                   # TO BE CREATED
        ├── CMakeLists.txt                # TO BE CREATED
        ├── config/
        │   └── nav2_params.yaml          # Core Smac, MPPI & Costmap configurations
        ├── launch/
        │   ├── path_planning.launch.py   # Main Nav2 bringup launcher
        │   └── rviz.launch.py            # Visualization launcher
        ├── rviz/
        │   └── nav2_default_view.rviz    # Saved RViz configuration
        ├── behavior_trees/
        │   └── custom_navigate.xml       # Custom Nav2 behavior tree
        └── src/
            └── costmap_bridge_node.cpp   # Converts perception topics to PointCloud2
```

---

## 3. Parallel Implementation To-Do List (For 2 Developers)

Because two developers (or AI instances) will be working on this simultaneously, the tasks are split into two parallel tracks. **Track A** focuses on core algorithms and launching, while **Track B** focuses on costmap integration and visualization.

### Shared Phase: Package Initialization
- [ ] **Task 0.1:** Create `package.xml` in `erc_path_planner/` with dependencies: `nav2_bringup`, `nav2_smac_planner`, `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_bt_navigator`, and `terrain_geometry_msgs`.
- [ ] **Task 0.2:** Update `CMakeLists.txt` to install the `config`, `launch`, `rviz`, and `behavior_trees` directories, and compile any C++ nodes.

#### 🔍 Validation Point 0 (Shared Checkpoint):
*   **Action:** Run `colcon build --packages-select erc_path_planner` from the workspace root.
*   **Success Criteria:** The build succeeds with no dependency errors, proving the package is correctly registered.

---

### Track A: Core Planners & Execution (Developer 1)
- [ ] **Task A.1 (Smac):** In `config/nav2_params.yaml`, configure the `planner_server` to load `nav2_smac_planner/SmacPlannerHybrid`. Set `motion_model_for_search: "REEDS_SHEPP"`, `minimum_turning_radius: 0.8`, and enable `allow_unknown: true`.
- [ ] **Task A.2 (MPPI):** Configure the `controller_server` to load `nav2_mppi_controller::MPPIController`. Set `batch_size: 2000`, `time_steps: 56`, `model_dt: 0.05`, and define velocity limits (`vx_max`, `vx_min`, `wz_max`). Configure critics (`GoalCritic`, `PathAlignCritic`, `ObstacleCritic`).
- [ ] **Task A.3 (Behavior Tree):** Configure the `bt_navigator` to use a standard XML tree, or create a custom `navigate_to_pose_w_replanning_and_recovery.xml` in the `behavior_trees/` folder.
- [ ] **Task A.4 (Launch):** Create `launch/path_planning.launch.py`. Include the standard `nav2_bringup` launcher and pass the `nav2_params.yaml` absolute path.

#### 🔍 Validation Point 1 (Track A Checkpoint):
*   **Action:** Run `ros2 run nav2_util lifecycle_bringup` and lint the YAML file. Then run `ros2 launch erc_path_planner path_planning.launch.py`.
*   **Success Criteria:** `planner_server` and `controller_server` transition to the `active` state in the terminal without crashing due to YAML misconfigurations.

---

### Track B: Costmaps, Perception & Vision (Developer 2)
- [ ] **Task B.1 (Static Map):** In `config/nav2_params.yaml`, configure `global_costmap` and `local_costmap` parameters to subscribe to the SLAM `/map` topic.
- [ ] **Task B.2 (Obstacles & Inflation):** Add an `obstacle_layer` to the `local_costmap`. Add an `inflation_layer` to both costmaps with a tuned `cost_scaling_factor` and `inflation_radius` corresponding to the rover's physical footprint.
- [ ] **Task B.3 (Bridge Node):** Write `src/costmap_bridge_node.cpp` to subscribe to the perception module's `terrain_geometry_msgs/ObstacleFeatureArray` and publish a standard `sensor_msgs/PointCloud2` for the `obstacle_layer` observation buffer.
- [ ] **Task B.4 (RViz2):** Create `launch/rviz.launch.py` to boot RViz2. Save a configuration to `rviz/nav2_default_view.rviz` that visually displays the Map, Global Costmap, Local Costmap, Global Plan, and MPPI Trajectories.

#### 🔍 Validation Point 2 (Track B Checkpoint):
*   **Action:** Build the bridge node. Run a test bag file containing perception data, launch the bridge node, and open RViz2.
*   **Success Criteria:** Dynamic obstacles appear as inflated red/blue lethal zones on the Costmap layer inside RViz2.

---

### Phase 4: Final System Integration
- [ ] **Task 4.1:** Both developers merge their work. Run the full path planning launch file alongside RViz2.
- [ ] **Task 4.2:** Publish a dummy 2D Pose Estimate (`/initialpose`) and a Nav2 Goal (`/goal_pose`) via RViz2.

#### 🔍 Final System Validation Point:
*   **Action:** Send a `/goal_pose` command in RViz2 while simulated or recorded obstacle data is playing.
*   **Success Criteria:** 
    1. A continuous global path (`/plan`) is drawn from the rover to the goal avoiding static walls.
    2. MPPI generates a valid velocity command (`/cmd_vel`) to follow the path.
    3. MPPI successfully deviates from the path locally if a dynamic rock appears in the Costmap.

---

## 4. Maintenance Notes
*   **Collaboration Rule:** Developer 1 (Track A) owns the Planner/Controller parameters and Launch files. Developer 2 (Track B) owns the Costmap parameters and C++ node development.
*   If a task is blocked, document the blocker below the task using blockquotes (`> Blocker: ...`).
