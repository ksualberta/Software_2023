from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_prams = os.path.join(get_package_share_directory('moveitservo'),'config','moveit_servo_config.yaml')

    moveitservonode = Node(
            package='moveit_servo',
            executable='servo_node_main',
            parameters=[moveit_prams],
         )

    return LaunchDescription([
        moveitservonode,       
    ])