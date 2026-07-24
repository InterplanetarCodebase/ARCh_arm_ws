import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_description_dir = get_package_share_directory("rover_arm_urdf")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    # Path to the plain URDF file (no xacro processing needed)
    urdf_file_path = os.path.join(arm_description_dir, "urdf", "rover_arm_urdf.urdf")

    controllers_yaml_path = os.path.join(arm_description_dir, "config", "ros2_controllers.yaml")

    def get_urdf_content():
        with open(urdf_file_path, 'r') as file:
            content = file.read()
        # gz_ros2_control needs a real filesystem path here, not a
        # package:// URI (it doesn't resolve those), so swap in the
        # actual resolved path at launch time.
        return content.replace("__ROS2_CONTROLLERS_YAML_PATH__", controllers_yaml_path)

    # Starts Gazebo Harmonic with an empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    # robot_state_publisher publishes /robot_description and TF. The URDF's
    # embedded <gazebo><plugin> tag (gz_ros2_control) is what actually spins
    # up controller_manager inside Gazebo once the entity is spawned.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(get_urdf_content(), value_type=str)
        }]
    )

    # Spawns the entity into the running Gazebo Harmonic world, reading the
    # URDF from the /robot_description topic.
    spawn_model_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_model",
        arguments=["-topic", "robot_description", "-name", "rover_arm_urdf"],
        output="screen"
    )

    # Controller spawners: these load and activate each controller defined
    # in ros2_controllers.yaml against controller_manager (which the URDF's
    # gz_ros2_control plugin already started inside Gazebo). Without these,
    # the interfaces exist but nothing is actually commanding them, so the
    # arm still collapses under gravity.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen"
    )

    # Bridges Gazebo's simulation clock onto the ROS2 /clock topic, so
    # controller_manager stops falling back to wall-clock time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # Chain the spawners one after another instead of firing all three at
    # once. controller_manager serializes these requests behind a lock,
    # and spawning them simultaneously was causing later ones to stall
    # waiting for the lock (or never activate at all).
    delayed_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_model_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delayed_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )

    delayed_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_model_node,
        clock_bridge,
        delayed_jsb,
        delayed_arm,
        delayed_gripper
    ])