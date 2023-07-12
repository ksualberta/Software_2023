from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():


    joystick_conv = TimerAction(
        period = 5.0,
        actions = [Node(
                package = 'teleop',
                executable = 'SPEAR_Arm_Node',
        )]
    )

    joy_input = Node(
                package = 'teleop',
                executable ='Joystick_Input',
    )

    return LaunchDescription([
           
        
        joy_input,
        joystick_conv
          
    ])