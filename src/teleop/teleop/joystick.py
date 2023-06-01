import rclpy
from rclpy import Node
from sensor_msgs import Joy


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        rclpy.create_node(Joy,'joystick_cmds', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        joy_msg = Joy()

        # Implement Joy Messages and controller input

        self.publisher.publish(joy_msg)
        

def main(args=None):

    rclpy.init(args=args)

    joystick_node = JoystickNode()

    rclpy.spin(joystick_node)

    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        