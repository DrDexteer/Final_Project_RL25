# Robotics Lab – Final Project (Fra2mo + Nav2/SLAM + iiwa + ArUco)

This repository contains the ROS 2 packages used for the Robotics Lab final project.
The setup includes:
- **Fra2mo** mobile robot navigation with **Nav2**
- **SLAM** using `slam_toolbox`
- **ArUco** detection pipeline (aruco_ros)
- **iiwa** bringup (from `ros2_iiwa`)
- A custom coordinator/logic package (**final_project_core**) to orchestrate behaviors

> Target ROS 2 distro: **Humble**  
> Simulation: Gazebo / Gazebo-ignition (depending on your setup)

---

## 1. Repository layout

Main packages in this repo:

- `ros2_fra2mo/`  
  Fra2mo robot description + Gazebo world/models + Nav2/SLAM configs + launch files.

- `final_project_core/`  
  High-level project logic (Python), e.g. coordinator node, interaction triggers, etc.


- `aruco_ros/`  
  ArUco detection stack (includes `aruco`, `aruco_ros`, `aruco_msgs`).


- `ros2_iiwa/`  
  iiwa bringup/description/controllers.

---

## 2. Setup and build

### 2.1 Requirements
Install  the main dependencies (typical):
- `nav2_bringup`
- `slam_toolbox`
- Gazebo + ROS-GZ bridge packages used by your system
- `rviz2`

> If something is missing, `rosdep` will tell you.

### 2.2 Build (recommended workspace layout)
Clone this repository inside a ROS 2 workspace, then build:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

All the commands below assume the workspace has been built and sourced.

---

## 3. Run in simulation

This project is typically run using **three terminals**:

- **Terminal 1**: bringup (Gazebo world + robots + bridges + coordinator logic)
- **Terminal 2**: Fra2mo **SLAM + Nav2 + RViz**
- **Terminal 3**: manual **requests** to drive the interaction pipeline

> In **each terminal**, make sure your workspace is sourced:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### 3.1 Terminal 1 — Bringup (world + robots + iiwa + coordinator)

Launch the simulation bringup:

```bash
ros2 launch final_project_core bringup_sim.launch.py
```

This launch is expected to start the Gazebo world (or ROS-GZ), spawn the robots, and bring up the high-level orchestration nodes (e.g., the coordinator).

---

### 3.2 Terminal 2 — Fra2mo SLAM + Nav2 + RViz

In a second terminal, start SLAM + navigation:

```bash
ros2 launch ros2_fra2mo fra2mo_slam.launch.py
```

Key parameter files used by the launch:
- SLAM params: `ros2_fra2mo/config/slam.yaml`
- Nav2 params: `ros2_fra2mo/config/navigation.yaml`
- RViz config: `ros2_fra2mo/rviz_conf/navigation.rviz`

---

### 3.3 Terminal 3 — Manual triggers (3 phases)

In a third terminal, you can drive the demo through the three phases using simple Bool topic publications.

#### Phase 1 — Start “interaction” (Fra2mo goes to the interaction pose + ArUco check)
```bash
ros2 topic pub --once /final_project_core/interaction_request std_msgs/msg/Bool "{data: true}"
```

#### Phase 2 — Manual “press” request (iiwa presses the button)
> Send this **only after** the system reports it is ready to press.
```bash
ros2 topic pub --once /final_project_core/manual_press_request std_msgs/msg/Bool "{data: true}"
```

#### Phase 3 — Enter room (Fra2mo enters the room: entry waypoint + final waypoint)
```bash
ros2 topic pub --once /final_project_core/enter_room_request std_msgs/msg/Bool "{data: true}"
```

---

### 3.4 (Optional) Quick status / debugging

Monitor the coordinator status messages:
```bash
ros2 topic echo /final_project_core/status
```

Check when the system is ready to accept the manual press:
```bash
ros2 topic echo /final_project_core/ready_to_press
```
