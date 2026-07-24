from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rover_arm_urdf", package_name="rover_arm_moveit_config").to_moveit_configs()
    
    ld = LaunchDescription()
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    rviz_ld = generate_moveit_rviz_launch(moveit_config)
    for entity in rviz_ld.entities:
        ld.add_action(entity)
        
    return ld
