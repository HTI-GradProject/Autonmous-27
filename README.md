# Autonomous Rover Simulation Workspace

This ROS 2 workspace contains packages for simulating and controlling the autonomous rover in the Mars Yard environment.

## Directory Structure

*   **[Rover/my_robot_description](Rover/my_robot_description)** - URDF model description, configurations, and spawn/launch scripts for the test rover.
*   **[Rover/worlds](Rover/worlds)** - The environment package containing Mars Yard and empty world definitions.
*   **[Rover/marsyard](Rover/marsyard)** - 3D models and configuration for the Mars Yard terrain.
*   **[MarsYardData](../MarsYardData)** - Datasets, elevation maps, and the `world1.world` environment file.

---

## Setup & Compilation

To build the workspace without keeping the output build artifacts in your Git repository, compile from **outside** the `Autonmous_Ws` folder (at the repository root directory where you cloned it):

```bash
# Go to the root repository directory (outside Autonmous_Ws)
cd /path/to/your/cloned/repository

# Sourcing standard ROS 2 (Humble)
source /opt/ros/humble/setup.bash

# Build the workspace packages
colcon build

# Source this workspace
source install/setup.bash
```

---

## 1. Launching the Rover into the World (One Command)

To launch the test rover spawned directly inside the Mars Yard simulation world, run the following single command:

```bash
ros2 launch my_robot_description gazebo.launch.py
```

> [!NOTE]
> By default, the launch file is configured to load `world1.world` and spawn the rover at a safe altitude (`z:=0.5`) to prevent collisions on startup.
>
> If you want to launch the rover in the **empty world** instead, run:
> ```bash
> ros2 launch my_robot_description gazebo.launch.py world:=empty_with_sensors.sdf
> ```

---

## 2. Launching the World Only (Without Rover)

To launch only the Mars Yard simulation world without spawning the rover, run:

```bash
ros2 launch worlds launch_map.launch.py world:=world1.world
```
