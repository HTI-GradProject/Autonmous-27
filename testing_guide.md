# Autonomous Rover Simulation & Perception System Testing Guide

This guide describes the complete workflow to build, launch, and validate the **Rover model (with RealSense D435i)**, the **Mars Yard world**, and the **Terrain Geometry (Perception) module**.

---

## 1. Prerequisites & Clean Workspace Rebuild

Ensure all background Gazebo/ROS nodes are terminated before proceeding. Run the compile steps from the root directory:

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

### Step B: Spawn the Rover (with RealSense D435i Camera)
Spawn the rover model containing the RGBD camera inside the running Mars Yard simulation:
```bash
ros2 launch my_robot_description gazebo.launch.py world:=world1.world
```
*   **Verification:** The rover model should appear on the Mars Yard terrain near the origin.

### Step C: Launch the Terrain Geometry Module (with Debug Topics)
Run the perception pipeline that processes raw point cloud data into costmaps and obstacle markers:
```bash
ros2 launch terrain_geometry terrain_geometry.launch.py publish_debug_topics:=true
```

### Step D: Run Teleop Twist Keyboard (Optional)
To drive the rover around and test dynamic updates:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*   Click inside this terminal window and use `u i o j k l m , .` keys to command rover velocities.

---

## 3. Telemetry & Topic Validation Checklist

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

---

## 4. Visual Verification inside RViz2

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

## 5. Troubleshooting & FAQ

*   **Error: `Unable to find uri[model://mars_yard]`**
    *   *Solution:* Make sure `/home/saif/Desktop/MESEKET/Autonmous-27/install/setup.bash` is sourced. The worlds launch script will automatically detect and resolve the local models path.
*   **Rover is sliding or physics behaves strangely in Gazebo:**
    *   *Reason:* DART physics uses small steps. If the real-time update rate is unstable, adjust Gazebo physics parameters inside `world1.world` under the `<physics>` tag.
*   **Perception costmap is empty in RViz2:**
    *   *Check:* Make sure the rover is close enough to rocks ($\le 6.0\text{ meters}$ range of camera depth sensor) and check that `/tf` transforms are active.
