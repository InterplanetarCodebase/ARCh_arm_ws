import os

import xacro
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_dir = get_package_share_directory('rover_arm_urdf')
    typing_dir = get_package_share_directory('rover_arm_typing')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo server-only (no GUI window)')

    world_path = os.path.join(typing_dir, 'worlds', 'typing_world.sdf')
    params_path = os.path.join(typing_dir, 'config', 'typing_params.yaml')
    controllers_yaml = os.path.join(arm_dir, 'config',
                                    'ros2_controllers.yaml')
    xacro_path = os.path.join(arm_dir, 'urdf',
                              'rover_arm_urdf_gazebo.urdf.xacro')

    # model:// resolution: rover_arm_urdf meshes resolve against the
    # install share dir; keyboard/table models against this package.
    resource_path = os.pathsep.join([
        os.path.dirname(arm_dir),                      # <prefix>/share
        os.path.join(typing_dir, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', resource_path)

    # Make sure the workspace-built (Harmonic-ABI) gz_ros2_control plugin
    # is found ahead of the broken Fortress-ABI one from apt.
    plugin_path = os.pathsep.join([
        os.path.join(get_package_prefix('gz_ros2_control'), 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ])
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path)

    def robot_description():
        content = xacro.process_file(xacro_path).toxml()
        return content.replace('__ROS2_CONTROLLERS_YAML_PATH__',
                               controllers_yaml)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression([
                "('-s -r ' if '", LaunchConfiguration('headless'),
                "' == 'true' else '-r ') + '", world_path, "'",
            ]),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description(),
                                                value_type=str),
            'use_sim_time': True,
        }],
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=['-topic', 'robot_description',
                   '-name', 'rover_arm_urdf'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # Controller spawners, chained (controller_manager serializes these
    # behind a lock; parallel spawns stall — same pattern as
    # rover_arm_urdf/launch/gazebo.launch.py).
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen')
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller'], output='screen')
    gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'], output='screen')

    chain = [
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_model, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner, on_exit=[arm_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner, on_exit=[gripper_spawner])),
    ]

    app_nodes = [
        Node(package='rover_arm_typing', executable='keyboard_detector',
             parameters=[params_path], output='screen'),
        Node(package='rover_arm_typing', executable='typing_controller',
             parameters=[params_path], output='screen'),
        Node(package='rover_arm_typing', executable='key_press_validator',
             parameters=[params_path], output='screen'),
    ]

    return LaunchDescription([
        headless_arg,
        set_resource_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_model,
        bridge,
        *chain,
        *app_nodes,
    ])
