from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    joystick_conv = Node(
            package = 'teleop',
            executable = 'SPEAR_Arm_Node',
    )

    joy_input = Node(
                package = 'teleop',
                executable ='Joystick_Input',
    )

    return LaunchDescription([
           
        joystick_conv,
        joy_input
          
    ])