import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher = self.create_publisher(Joy,'joystick_cmds', 10)

        pygame.init()
        pygame.joystick.init()
            
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    """
    Description: Intializes pygame, and joystick. Continuously Checks for Joystick input and warns user if no input detected.
    args: None
    Return: None
    """
    def timer_callback(self):

        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init
        else:

            self.joystick = None
            self.get_logger().warn('No Joystick Detected')
            return

        
        self.joy_msg = Joy()

        #Gets data from Joystick and converts to Joy msg.
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.joy_msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        self.joy_msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        
        #Testing purposes, publishes message to logs
        self.call_log_msgs

        #Publishes Message
        self.publisher.publish(self.joy_msg)

    def call_log_msgs(self):

        self.get_logger().info('Publishing header: "%s"' % self.joy_msg.header.stamp)
        self.get_logger().info('Publishing axes: "%s"' % str(self.joy_msg.axes))
        self.get_logger().info('Publishing buttons: "%s"' % str(self.joy_msg.buttons))
       
        

def main(args=None):

    rclpy.init(args=args)

    joystick_node = JoystickNode()

    rclpy.spin(joystick_node)

    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        