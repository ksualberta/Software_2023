import rclpy
from rclpy import Node
from sensor_msgs import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        rclpy.create_node(Joy,'joystick_cmds', 10)


        