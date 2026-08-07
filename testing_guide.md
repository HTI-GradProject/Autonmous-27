# Autonomous Rover Simulation & Perception System Testing Guide

This guide describes the complete workflow to build, launch, and validate the **Rover model (with RealSense D435i)**, the **Mars Yard world**, and the **Terrain Geometry (Perception) module**.

---

## 1. Prerequisites & Clean Workspace Rebuild

Ensure all background Gazebo/ROS nodes are terminated before proceeding. Run the cleanup command to force-kill any running instances:

```bash
pkill -9 -f gazebo; pkill -9 -f ign; pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; killall -9 -q ruby gz server rviz2 parameter_bridge ros2 robot_state_publisher
```

Run the compile steps from the root directory:

```bash
# Navigate to the workspace root
cd /home/saif/Desktop/MESEKET/Autonmous-27

# Clear previous build targets to prevent cache pollution
rm -rf build/ install/ log/

# Run clean colcon compilation
colcon build --symlink-install

# Source the ROS 2 Humble environment and local workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 2. Step-by-Step System Launch Procedure

Execute each of the following commands in **separate terminal windows** (making sure to source `/opt/ros/humble/setup.bash` and `source install/setup.bash` in each new terminal).

### Step A: Launch the Simulation World
Launch the Mars Yard world with obstacles (rocks scaled up to 2.5x):
```bash
ros2 launch worlds world1.launch.py
```
*   **Verification:** You should see Gazebo Sim open showing the red Martian terrain with several medium/large rocks spawned at varying coordinates.
*   **How to Kill:** Press `Ctrl + C` in the terminal window. If Gazebo remains open or frozen in the background, run:
    ```bash
    pkill -f gz
    ```

### Step B: Spawn the Rover (with RealSense D435i Camera)
Spawn the rover model containing the RGBD camera inside the already running Mars Yard simulation:
```bash
ros2 launch my_robot_description spawn_rover.launch.py
```
*   **Verification:** The rover model should appear on the Mars Yard terrain near the origin.
*   **How to Kill:** Press `Ctrl + C` in the terminal. To clean up all ROS-Gazebo bridge processes and state publishers spawned by this node, run:
    ```bash
    pkill -f parameter_bridge
    pkill -f robot_state_publisher
    ```

> [!NOTE]
> * **Independent Spawning:** `spawn_rover.launch.py` only spawns the robot and establishes topic bridges; it expects Gazebo to be running first (via `world1.launch.py`).
> * **Combined Spawning:** If you want a single command that launches BOTH the world and the rover together, run `ros2 launch my_robot_description gazebo.launch.py world:=world1.world`.

### Step C: Launch the Terrain Geometry Module (with Debug Topics)
Run the perception pipeline that processes raw point cloud data into costmaps and obstacle markers:
```bash
ros2 launch terrain_geometry terrain.launch.py publish_debug_topics:=true
```
*   **Verification:** Check the log output for `Frame processed | in=...` messages, indicating active point cloud processing.
*   **How to Kill:** Press `Ctrl + C` in the terminal. To manually terminate the node process directly if running in the background:
    ```bash
    pkill -f terrain_node
    ```

> [!NOTE]
> **Automatic TF Resolution:** The camera sensor in Gazebo publishes point clouds under the frame ID `my_robot/camera_link/camera`. The launch scripts (`spawn_rover.launch.py` and `gazebo.launch.py`) now automatically launch a `static_transform_publisher` to bridge `camera_link` to `my_robot/camera_link/camera`, preventing TF lookup failures.

### Step D: Teleoperate and Drive the Rover (Two Choices)
To drive the rover around the Mars Yard and verify costmap / obstacle tracking updates in real-time, choose one of these options:

#### Choice 1: Launch the Custom Teleoperation GUI (Recommended)
This opens a dark-themed visual desktop GUI with velocity control sliders, click buttons, and arrow-key driving support:
```bash
ros2 run my_robot_description teleop_gui.py
# Or run the script directly:
python3 Autonmous_Ws/Rover/my_robot_description/scripts/teleop_gui.py
```
*   **Controls:** Click the **Forward / Reverse / Left / Right** buttons, use your keyboard's **Arrow Keys** to steer, or press **Space** to halt instantly. Adjust maximum linear and angular speeds dynamically using the sliders!
*   **How to Kill:** Close the GUI window, or press `Ctrl + C` in the launching terminal.

#### Choice 2: Keyboard Teleop (Standard ROS Terminal Tool)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*   **Controls:** Click inside the terminal window and use `u i o j k l m , .` keys to command rover velocities.
*   **How to Kill:** Press `Ctrl + C` in the terminal window.

---

## 3. Global System Clean Up Command (Emergency Kill)

If any simulator, bridge, or node processes remain running or block ports in the background, you can terminate all ROS and Gazebo related processes instantly in a single command:

```bash
pkill -f gz ; pkill -f ros
```

---

## 4. Telemetry & Topic Validation Checklist

Open a new terminal to inspect the active data streams and verify that sensors and processing nodes are working correctly.

### Check 1: Verify Camera Sensors are Publishing
Ensure that the simulated D435i depth camera is outputting raw points and images:
```bash
# Check if point cloud topic is active and publishing data
ros2 topic hz /camera/depth/color/points

# Check if raw camera image topic is active
ros2 topic hz /camera/image_raw
```
*   *Expected Result:* Both topics should publish at $\approx 10\text{--}30\text{ Hz}$.

### Check 2: Verify Perception Module Outputs
Confirm that the terrain geometry node is processing the cloud and outputting obstacle metrics:
```bash
# Verify costmap grid publications
ros2 topic hz /terrain/costmap

# Print detected obstacle centroids and sizes
ros2 topic echo /terrain/obstacle_features
```
*   *Expected Result:* `/terrain/costmap` is published at $\approx 2\text{--}5\text{ Hz}$. You should see arrays of coordinate centroids and bounding dimension features corresponding to the obstacles.

### Check 3: Inspecting Active Nodes, Topics, and Connections

To understand exactly what is running in the ROS 2 graph and verify connections between components:

- **List all active nodes:**
  ```bash
  ros2 node list
  ```
  *Expected Nodes:* You should see `/terrain_node`, `/robot_state_publisher`, `/static_transform_publisher` (instances bridging frames), and `/parameter_bridge` nodes.

- **List all active topics:**
  ```bash
  ros2 topic list
  ```
  *Expected Topics:* `/camera/depth/color/points`, `/terrain/costmap`, `/terrain/obstacle_features`, `/terrain/obstacle_markers`, `/cmd_vel`, `/odom`, and `/clock`.

- **Show detailed info about a topic:**
  Inspect details like topic publishers, subscription counts, and message types:
  ```bash
  ros2 topic info /terrain/costmap --show-types
  ```

- **Show detailed info about a node:**
  Query a node's active subscriptions, publishers, service servers, and service clients:
  ```bash
  ros2 node info /terrain_node
  ```

- **Graphically view the node connection map (Recommended):**
  Open the ROS graph visualization utility to see data flows between nodes:
  ```bash
  rqt_graph
  ```
  *Verification:* This opens a graphical UI showing the node network. You should verify `/terrain_node` subscribing to `/camera/depth/color/points` and publishing to `/terrain/costmap`, `/terrain/obstacle_features`, and `/terrain/obstacle_markers`.

---

## 5. Visual Verification inside RViz2

To graphically visualize the rover, raw camera data, and the perception outputs:

1.  Open RViz2:
    ```bash
    rviz2
    ```
2.  Set the **Fixed Frame** (in the Global Options panel on the left) to **`base_link`** or **`odom`**.
3.  Add the following displays (using the **Add** button at the bottom-left):

| Display Type | Topic | Description / What to look for |
| :--- | :--- | :--- |
| **RobotModel** | *(none)* | Renders the 3D rover model mesh structure. |
| **TF** | *(none)* | Shows coordinate axis transforms (`base_link` $\rightarrow$ `camera_link`). |
| **PointCloud2** | `/camera/depth/color/points` | Renders raw 3D colored point cloud points from the RealSense camera. |
| **Image** | `/camera/image_raw` | Shows the camera feed overlay. |
| **OccupancyGrid** | `/terrain/costmap` | Displays the costmap plane (cost pixels representing safe/unsafe cells). |
| **MarkerArray** | `/terrain/obstacle_markers` | Shows bounding boxes surrounding each rock with labeled cluster IDs. |
| **PointCloud2** (Debug) | `/terrain/debug/ground_cloud` | Shows ground points removed by the plane removal algorithm (colored in green/gray). |
| **PointCloud2** (Debug) | `/terrain/debug/clustered_cloud` | Shows remaining obstacle points grouped and colored by their cluster IDs. |

---

## 6. Troubleshooting & FAQ

*   **Error: `Unable to find uri[model://mars_yard]`**
    *   *Solution:* Make sure `/home/saif/Desktop/MESEKET/Autonmous-27/install/setup.bash` is sourced. The worlds launch script will automatically detect and resolve the local models path.
*   **Rover is sliding or physics behaves strangely in Gazebo:**
    *   *Reason:* DART physics uses small steps. If the real-time update rate is unstable, adjust Gazebo physics parameters inside `world1.world` under the `<physics>` tag.
*   **Perception costmap is empty or warnings say TF lookup failed for `my_robot/camera_link/camera`:**
    *   *Reason:* The Gazebo sensor publishes point clouds using the frame ID `my_robot/camera_link/camera`, while TF publishes transforms for `camera_link`.
    *   *Solution:* We have integrated the `static_transform_publisher` directly into `spawn_rover.launch.py` and `gazebo.launch.py` to bridge these frames automatically. If you are playing back a ROS bag or need to launch it manually, run:
        ```bash
        ros2 run tf2_ros static_transform_publisher --frame-id camera_link --child-frame-id my_robot/camera_link/camera --ros-args -p use_sim_time:=true
        ```
    *   *Check:* Also, make sure the rover is close enough to rocks ($\le 6.0\text{ meters}$ range of camera depth sensor) and check that `/tf` transforms are active.
*   **Error: `Package 'terrain_geometry' not found`:**
    *   *Reason 1:* The package was sourced in a terminal session before it successfully finished building.
    *   *Reason 2:* The shell environment variable cache is corrupted by sourcing `setup.bash` multiple times in the same session.
    *   *Solution:* Reset your terminal session by running `exec bash`, source the base setup (`source /opt/ros/humble/setup.bash`), and then source the local setup (`source install/setup.bash`).
*   **Warning: Colcon identifies `terrain_geometry` as `(python)` instead of `(ros.ament_python)`:**
    *   *Reason:* An invalid XML comment containing a double hyphen (`--`) is present in `package.xml`. Double hyphens are illegal inside XML comments.
    *   *Solution:* Replace `--` with a single `-` in `package.xml`, clear the build directories (`rm -rf build/ install/ log/`), and rebuild the workspace.
