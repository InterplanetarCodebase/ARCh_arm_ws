import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    arm_description_dir = get_package_share_directory("rover_arm_urdf")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Path to the plain URDF file (no xacro processing needed)
    urdf_file_path = os.path.join(arm_description_dir, "urdf", "rover_arm_urdf.urdf")

    # Mirrors <include file="$(find gazebo_ros)/launch/empty_world.launch" />
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        )
    )

    # Mirrors the ROS1 tf_footprint_base static_transform_publisher node.
    # In ROS2, static_transform_publisher is latched by default, so the
    # ROS1 publish-rate argument (40) is dropped.
    tf_footprint_base_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_footprint_base",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    # Mirrors spawn_model. gazebo_ros's ROS1 spawn_model executable is
    # replaced by spawn_entity.py in ROS2, and the -model flag becomes -entity.
    spawn_model_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_model",
        arguments=["-file", urdf_file_path, "-urdf", "-entity", "rover_arm_urdf"],
        output="screen"
    )

    # Mirrors "rostopic pub /calibrated std_msgs/Bool true". rostopic has
    # no ROS2 node equivalent, so this uses the ros2 CLI via ExecuteProcess.
    fake_joint_calibration = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "--once", "/calibrated", "std_msgs/msg/Bool", "{data: true}"],
        name="fake_joint_calibration",
        output="screen"
    )

    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base_node,
        spawn_model_node,
        fake_joint_calibration
    ])
