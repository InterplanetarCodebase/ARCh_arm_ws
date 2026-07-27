## Launching the Simulation (ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2)

Running the full simulation — trajectory controllers, motion planning, and visualization — requires **four components running in parallel**, each in its own terminal.

> **Before every terminal:** source your workspace.
> ```bash
> source install/setup.bash
> ```

---

### Terminal 1 — Gazebo + ROS 2 Controllers

```bash
ros2 launch rover_arm_urdf gazebo.launch.py
```

Wait for Gazebo to open and the arm to spawn. A `Successfully switched controllers` message in the terminal confirms it's ready.

---

### Terminal 2 — Simulation Clock Bridge

Gazebo Harmonic doesn't publish its simulation clock to ROS 2 automatically — this bridge is required, or **MoveIt will freeze**.

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

---

### Terminal 3 — MoveIt 2 Planner (the "brain")

```bash
ros2 launch rover_arm_moveit_config move_group.launch.py
```

---

### Terminal 4 — RViz (Interface)

```bash
ros2 launch rover_arm_moveit_config moveit_rviz.launch.py
```

---

### Usage

Once RViz opens:

1. Drag the interactive marker on the gripper to your desired pose.
2. Click **Plan and Execute** in the MoveIt panel.

### Startup Order Matters

| Order | Terminal | Component |
|-------|----------|-----------|
| 1 | 1 | Gazebo + Controllers |
| 2 | 2 | Clock Bridge |
| 3 | 3 | MoveIt 2 Planner |
| 4 | 4 | RViz |

Launching out of order (especially skipping the clock bridge before MoveIt) is the most common cause of a frozen planner.
