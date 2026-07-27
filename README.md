Markdown

## Launching the Simulation (ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2)

To run the full simulation with trajectory controllers and motion planning, you need to launch four components. 

Open four separate terminals. In every terminal, make sure to source your workspace first:
`source install/setup.bash`

**Terminal 1: Start Gazebo and the ROS 2 Controllers**
```bash
ros2 launch rover_arm_urdf gazebo.launch.py

(Wait for Gazebo to open and the arm to spawn. You should see "Successfully switched controllers" in the terminal.)

Terminal 2: Bridge the Simulation Clock
Gazebo Harmonic requires a manual bridge to publish its internal simulation clock to the ROS 2 network. Without this, MoveIt will freeze.
Bash

ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

Terminal 3: Start the MoveIt 2 Planner (Brain)
Bash

ros2 launch rover_arm_moveit_config move_group.launch.py

Terminal 4: Start RViz (User Interface)
Bash

ros2 launch rover_arm_moveit_config moveit_rviz.launch.py

Once RViz opens, drag the interactive marker on the gripper to your desired pose and click Plan and Execute in the MoveIt panel.
