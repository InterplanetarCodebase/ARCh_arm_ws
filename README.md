# Rover Arm — ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2

Simulation and workspace-analysis tooling for the 6-DOF rover arm (`rover_arm_urdf` / `rover_arm_moveit_config`).

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2
- Python 3 with `venv`

Every terminal below must first source the workspace:

```bash
source install/setup.bash
```

---

## 1. Running the Full Simulation

Four components run in parallel, each in its own sourced terminal, **started in this exact order**:

| # | Terminal | Component | Why order matters |
|---|----------|-----------|--------------------|
| 1 | Gazebo + Controllers | Spawns the arm and starts `ros2_control` | Must exist before anything can bridge to it |
| 2 | Clock Bridge | Publishes Gazebo's sim clock to ROS 2 | **MoveIt will freeze without this** |
| 3 | MoveIt 2 Planner | The planning "brain" | Needs the clock to be ticking first |
| 4 | RViz | Visualization + interactive markers | Purely a client of the above three |

Skipping or reordering steps — especially bringing up MoveIt before the clock bridge — is the most common cause of a frozen planner.

### Terminal 1 — Gazebo + ROS 2 Controllers

```bash
ros2 launch rover_arm_urdf gazebo.launch.py
```

Wait for Gazebo to open and the arm to spawn. A `Successfully switched controllers` message confirms it's ready.

### Terminal 2 — Simulation Clock Bridge

Gazebo Harmonic doesn't publish its sim clock to ROS 2 automatically:

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Terminal 3 — MoveIt 2 Planner

```bash
ros2 launch rover_arm_moveit_config move_group.launch.py
```

### Terminal 4 — RViz

```bash
ros2 launch rover_arm_moveit_config moveit_rviz.launch.py
```

### Usage

1. In RViz, drag the interactive marker on the gripper to the desired pose.
2. Click **Plan and Execute** in the MoveIt panel.

---

## 2. Workspace Analysis Tools

Two ways to analyze and visualize the reachable 3D workspace of the arm via Monte Carlo forward kinematics.

### Option A — Matplotlib 3D & 2D Cross-Section Plots

Standalone, no ROS environment needed. Produces a multi-panel dashboard: 3D point cloud, top-down swing radius, and side-profile reach vs. height.

```bash
python3 -m venv venv_plot
source venv_plot/bin/activate
pip install ikpy matplotlib scipy numpy
python3 workspace_plot.py
```

Exit the environment when done:

```bash
deactivate
```

### Option B — Live RViz2 Point Cloud Publisher

Renders the reachable workspace as a colored 3D point cloud wrapped around the robot model, live in RViz2.

**Setup (one-time):**

```bash
python3 -m venv --system-site-packages venv_ros
source venv_ros/bin/activate
pip install ikpy matplotlib numpy
```

**Terminal 1 — Launch the MoveIt simulation environment:**

```bash
cd ~/rover_arm_ws
source install/setup.bash
ros2 launch rover_arm_moveit_config demo.launch.py
```

**Terminal 2 — Run the publisher node:**

```bash
cd ~/rover_arm_ws
source venv_ros/bin/activate
python3 workspace_publisher.py
```

**In RViz2:**

1. Click **Add** at the bottom of the Displays panel.
2. Select **Marker**.
3. Set its Topic to `/workspace_cloud`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| MoveIt planner appears frozen / never plans | Clock bridge (Terminal 2) not running, or started after MoveIt | Kill and restart in the correct order: Gazebo → Clock Bridge → MoveIt → RViz |
| RViz opens but no robot model / TF errors | Gazebo (Terminal 1) not fully spawned before RViz launch | Wait for `Successfully switched controllers` before starting later terminals |
| `ikpy` import errors in `venv_ros` | Missing `--system-site-packages` flag when creating the venv | Recreate the venv with `python3 -m venv --system-site-packages venv_ros` |
| No point cloud in RViz2 | Marker display not added, or wrong topic | Add a **Marker** display and confirm topic is `/workspace_cloud` |
