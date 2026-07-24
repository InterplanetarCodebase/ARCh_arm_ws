import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_description_dir = get_package_share_directory("rover_arm_urdf")

    # Path to the plain URDF file (no xacro processing needed)
    urdf_file_path = os.path.join(arm_description_dir, "urdf", "rover_arm_urdf.urdf")

    # Mirrors the <arg name="model" /> from the ROS1 launch file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=urdf_file_path,
        description="Absolute path to robot urdf file"
    )

    # Read the URDF content directly and feed it to robot_description,
    # replacing the ROS1 <param textfile="..."> mechanism.
    def get_urdf_content():
        with open(urdf_file_path, 'r') as file:
            return file.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(get_urdf_content(), value_type=str)
        }]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(arm_description_dir, "rviz", "urdf_config.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
