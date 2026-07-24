# ARCh_arm_ws

ROS 2 Humble workspace for the 5-DOF arm of the Australian Rover
Challenge 2027 rover: inverse kinematics, Gazebo Harmonic simulation, and
an autonomous keyboard-typing stack guided by a wrist-mounted RGB-D
camera.

## Packages

| Package | Purpose |
|---|---|
| `rover_arm_urdf` | Robot description (URDF, meshes), wrist RGB-D camera + `tool_tip` frames, Gazebo launch |
| `rover_arm_moveit_config` | MoveIt 2 config (Setup Assistant) — RViz IK sandbox with mock hardware |
| `rover_arm_typing` | Autonomous typing stack: keyboard detection, analytic IK controller, validator, sim world |
| `gz_ros2_control_src` | Source checkout of `gz_ros2_control` (humble branch) built for Gazebo **Harmonic** — the apt package is built against Fortress and cannot load |

## Requirements

- ROS 2 Humble + Gazebo Harmonic (`gz-sim8`) with the
  `ros-humble-ros-gzharmonic-*` bridge packages
- MoveIt 2 (`rosdep install --from-paths src --ignore-src -r -y`;
  `warehouse_ros_mongo` is unavailable on Jammy and safe to skip)
- OpenCV Python with `cv2.aruco` (pip `opencv-python` works)

## Build

```bash
cd ~/Documents/ARCh_arm_ws
source /opt/ros/humble/setup.bash
GZ_VERSION=harmonic colcon build --symlink-install
source install/setup.bash
```

`GZ_VERSION=harmonic` matters: it makes `gz_ros2_control` compile against
`gz-sim8` so its plugin actually loads inside Gazebo Harmonic.

## Run

```bash
# URDF viewer (sliders + RViz, no physics)
ros2 launch rover_arm_urdf display.launch.py

# MoveIt 2 demo (RViz interactive IK, mock hardware)
ros2 launch rover_arm_moveit_config demo.launch.py

# Plain Gazebo sim (arm + controllers, empty world)
ros2 launch rover_arm_urdf gazebo.launch.py

# Autonomous keyboard-typing simulation
ros2 launch rover_arm_typing typing_sim.launch.py          # with GUI
ros2 launch rover_arm_typing typing_sim.launch.py headless:=true
```

Then, in another terminal:

```bash
source install/setup.bash
ros2 topic pub --once /type_text std_msgs/msg/String "{data: 'HELLO WORLD'}"
ros2 topic echo /typed_keys        # characters as the validator confirms them
ros2 topic echo /typing_status     # controller state machine progress
```

## Typing stack architecture

```
wrist RGB-D camera (gz sensor, 1280x960)
        │ ros_gz bridge: /wrist_camera/{image,depth_image,camera_info}
        v
keyboard_detector ── ArUco corners → homography → per-key depth → TF to base_link
        │ /keyboard/key_map (JSON, latched)   /keyboard/markers (RViz)
        v
typing_controller ── /type_text → scan pose → per char: analytic IK
        │             (hover → press → retract) via
        │             /arm_controller/follow_joint_trajectory
        v
key_press_validator ── watches TF tool_tip vs key map → /typed_keys, PASS/FAIL
```

Key design points:

- **Analytic IK** (`rover_arm_typing/ik.py`): the arm has 5 DOF (base
  roll + 3 one-sided pitches + wrist roll), so full 6-DOF pose IK is
  over-constrained. Typing needs position + tool-straight-down = exactly
  5 constraints, solved in closed form (base yaw + planar 2R +
  wrist-pitch completion). Hover poses may lean the tool back by up to
  0.3 rad where the one-sided wrist-pitch limit demands it.
- **Detection** (`keyboard_detector.py`): four ArUco markers
  (DICT_4X4_50, ids 0–3) printed on the keyboard corners give a
  homography from the layout frame (`config/key_layout.yaml`, generated
  together with the texture by `texture_gen.py`) to the image; each key
  centre is then depth-sampled and deprojected — the same code would run
  on a real printed keyboard.
- **Validation** is proximity-based (no physically depressing keys):
  the validator emits a key when `tool_tip` dips into a small cylinder
  above it.

Verified end-to-end headless: `ARC` and `HELLO WORLD` both typed with
every press within ~7 mm of the detected key centre (validator PASS).

## Notes / gotchas

- The plain `rover_arm_urdf.urdf` is simulation-agnostic; Gazebo-only
  bits (gz_ros2_control plugin, camera sensor) live in
  `rover_arm_urdf_gazebo.urdf.xacro`. Don't re-add `<ros2_control>` to
  the plain URDF — MoveIt includes it and would try to load the Gazebo
  hardware plugin outside Gazebo.
- gz-sim renders a box's top-face texture rotated 90°; the keyboard
  texture is pre-rotated in `texture_gen.py` to compensate. If you
  regenerate it, keep that step.
- If RViz dies with a `GLIBC_PRIVATE`/`libpthread` symbol error, you are
  in a snap-polluted terminal (VS Code snap) — run from a regular
  terminal.
- Kindly remove `/build /install /log` before committing.
