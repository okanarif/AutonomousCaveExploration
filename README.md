https://github.com/user-attachments/assets/6dba888a-013f-4a5f-9c72-589300eedba9

# Autonomous Cave Exploration (ROS 2)

This repository contains a ROS 2-based stack for **autonomous cave exploration** with a quadrotor.

High-level pipeline:

- **Simulation**: Unity-based simulator + sensor/TF bridges
- **Perception / Mapping**: Depth → PointCloud → **OctoMap**
- **Mission management (FSM)**: IDLE → TAKEOFF → NAVIGATE_TO_CAVE → EXPLORE_CAVE → FINISHED
- **Frontier detection**: 3D frontier extraction and goal selection on OctoMap
- **Trajectory generation**: Sampling-based, collision-free quintic trajectory generation
- **Lantern detection**: Lantern detection/localization using semantic + depth cameras

> Note: This repo is organized as a ROS 2 workspace under `ros2_ws/`.

---

## Demo video

- Download/view: [Video.mp4](Video.mp4)

If your GitHub view supports HTML video embeds, it should play inline:

<video src="Video.mp4" controls></video>

---

## Workspace structure

- `ros2_ws/src/`
	- `cave_navigation`: Meta-package + integrated launch files (one-command run)
	- `simulation`: Unity/ROS bridge nodes and simulation launch files
	- `octomap_mapping/octomap_server`: OctoMap server + mapping launch files
	- `mission_control`: Mission FSM node
	- `frontier_detector`: 3D frontier detector node
	- `trajectory_generator`: Sampling-based trajectory planner node
	- `lantern_detection`: Lantern detection node
	- `controller`: Quadrotor tracking controller
	- `mission_interfaces`: Custom service interfaces (`ExecuteTrajectory`)

---

## Requirements

- Linux (Ubuntu 22.04 + ROS 2 **Humble** is the most common target; Jazzy/Rolling may also work)
- ROS 2 + `colcon`
- `rosdep`
- Build tools: C++17 toolchain, CMake

The `simulation` package fetches `libsocket` from GitHub at build time (`ExternalProject_Add`), so the first build requires internet access.

---

## Setup (dependencies)

After installing your ROS 2 environment:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update

cd ros2_ws
rosdep install --from-paths src -y --ignore-src
```

---

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Quick start (full mission)

To start the full stack with a single command:

```bash
source ros2_ws/install/setup.bash
ros2 launch cave_navigation integrated_mission.launch.py rviz:=true
```

This launch includes:

- `simulation/launch/simulation.launch.py`
- `octomap_server/launch/perception_mapping.launch.py`
- `mission_control_node`
- `cave_navigation/launch/cave_exploration.launch.py` (frontier + trajectory)
- `lantern_detection/launch/lantern_detection.launch.py`
- Optional `rviz2` (`rviz:=true/false`)

### Important note: Unity simulation binary

Inside `simulation.launch.py`, the following executable is also launched:

- `Simulation.x86_64` under the `simulation` package

This file is not present in this repository. If you have a Unity build, copy it to the following location after building and make it executable:

```bash
cp /path/to/Simulation.x86_64 ros2_ws/install/simulation/lib/simulation/Simulation.x86_64
chmod +x ros2_ws/install/simulation/lib/simulation/Simulation.x86_64
```

If you don’t have this binary, you can still run the other parts of the system individually (see below).

---

## Modular runs (component-wise)

### 1) Mapping (Depth → PointCloud → OctoMap)

```bash
source ros2_ws/install/setup.bash
ros2 launch octomap_server perception_mapping.launch.py
```

This launch uses `depth_image_proc/point_cloud_xyz_node` to generate a pointcloud from `/realsense/depth/*` topics and feeds it into `octomap_server_node`.

### 2) Exploration stack (Frontier + Trajectory)

```bash
source ros2_ws/install/setup.bash
ros2 launch cave_navigation cave_exploration.launch.py
```

### 3) Mission control (FSM)

```bash
source ros2_ws/install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

Mission control calls the `start_navigation` service to navigate to the cave entrance using the configured waypoints, and during exploration it integrates with the frontier detector via the `start_exploration` service.

### 4) Lantern detection

```bash
source ros2_ws/install/setup.bash
ros2 launch lantern_detection lantern_detection.launch.py use_sim_time:=true
```

### 5) Controller (standalone)

```bash
source ros2_ws/install/setup.bash
ros2 launch controller controller.launch.py
```

---

## Interfaces (topics / services)

### Mission FSM state

- `stm_mode` (`std_msgs/String`): publishes the FSM state (e.g., `IDLE`, `TAKEOFF`, `NAVIGATE_TO_CAVE`, `EXPLORE_CAVE`, `FINISHED`).

### Trajectory start service

- Service name (default): `start_navigation`
- Type: `mission_interfaces/srv/ExecuteTrajectory`
- Request: `geometry_msgs/PoseArray waypoints`
- Response: `bool success`, `string message`

This service is provided by `trajectory_generation_node` in `trajectory_generator`, and called as a client by `mission_control_node`.

### Exploration start service

- Service name (default): `start_exploration`
- Type: `std_srvs/Trigger`
- Server: `frontier_detector`

---

## Configuration

Parameters are loaded from YAML files:

- `ros2_ws/src/mission_control/config/mission_control_params.yaml`
	- Takeoff altitude, tolerances, cave-entrance waypoints
- `ros2_ws/src/trajectory_generator/config/trajectory_generator_params.yaml`
	- Sampling budget, speeds, collision checking, topic/service names
- `ros2_ws/src/frontier_detector/config/frontier_detector_params.yaml`
	- Frontier filters, mean-shift parameters, goal publish frequency
- `ros2_ws/src/lantern_detection/config/lantern_detection_params.yaml`
	- Semantic/depth topics, thresholds, and frames

---

## Helper scripts

`ros2_ws/scripts/takeoff.py`:
Publishes a simple takeoff command by sending a `desired_state` 2 m above the current position.

```bash
source ros2_ws/install/setup.bash
python3 ros2_ws/scripts/takeoff.py
```

`ros2_ws/scripts/waypoint_generator_start_to_cave_entrance.py`:
Generates interpolated/sampled waypoints from example points (offline helper). Requires `numpy/matplotlib`.

---

## Troubleshooting

- If `start_navigation` / `start_exploration` services are not visible:
	- Verify the relevant nodes are running: `ros2 node list`
	- Check whether you changed service names in the YAML configs.
- If no OctoMap is being produced:
	- Verify `/realsense/depth/image` and `/realsense/depth/camera_info` topics are being published.
	- Ensure `depth_image_proc` is installed (usually handled by `rosdep`).
- If the simulator does not start:
	- Ensure `Simulation.x86_64` exists under `ros2_ws/install/simulation/lib/simulation/` and is executable (`chmod +x`).

