from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

param_file = os.path.join(get_package_share_directory("moveit_config"), "config", "moveit_servo_config.yaml")
    
print("ParamFile:" + param_file)

def generate_launch_description():

    ld = LaunchDescription()
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name='arm_servo',
        parameters= [param_file]
        
    )
 
    ld.add_action(servo_node)
    return ld