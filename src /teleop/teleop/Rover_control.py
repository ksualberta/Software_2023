import rclpy
from rclpy.node import Node
from std_msgs import Float32MultiArray
from sensor_msgs import JointState
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
import time

drive_topic = "/Rover/drive_commands
steer_topic = "/Rover/steer_commands"
joy_topic = "/Rover/Joy_Topic"

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
LEFT_TRIGGER = 2
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5
D_PAD_X = 6
D_PAD_Y = 7
A = 0
B = 1
X = 2
Y = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
CHANGE_VIEW = 6
MENU = 7
HOME = 8
LEFT_STICK_CLICK = 9
RIGHT_STICK_CLICK = 10

#Note Motor Mapping 

"""
0 3
1 4
2 5
"""

class RoverControl(Node):

    def __init__(self):
        super().__init__('rover_control')

        #Publsiher drive and steer commands
        self.drive_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = drive_topic, qos_profile = QoSProfile(depth=10))
        self.steer_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = steer_topic, qos_profile = QoSProfile(depth=10))

        #Subscription to Joy commands         
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = joy_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)

        timer_period = 0.2

        self.last_toggle_time = time.time()
        self.steer_lock_state = True
        self.toggle_debounce_time = 0.5 #debounce time
        self.drive_motors_num = 6 #Number of drive motors
        self.steer_motors_num = 4 #Number of steering motors

        self.steer_right_limit = 0.5
        self.steer_left_limit = 2.64
        self.steer_center = (steer_right_limit + steer_left_limit)/2

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.rover_main_control)


    def JoystickMsg(self, msg):
        
        self.joystick_msg = msg
    
    def rover_main_control(self):
        self.drive_cmds_msg = Float32MultiArray()
        self.steer_cmds_msg = Float32MultiArray()

        #Number of Drive Motors and Steering Motors
        
        self.drive_cmds_msg.data = [0.0]* self.drive_motors_num
        self.steer_cmds.data = [self.steer_center]* self.steer_motors_num

        if joystick_msg.buttons[RIGHT_BUMPER] or joystick_msg.buttons[LEFT_BUMPER]

            self.skid_steer_mode()

        else:

            self.drive_motor_conv()

        self.drive_cmds.publish(self.drive_cmds_msg)

        
        if(joystick_msg.buttons[MENU]):
            steer_toggle()

        if steer_lock_state:

            steer_motor_conv_locked()

        elif not steer_lock_state:

            steer_motor_conv_unlocked():

        self.steer_cmds.publish(steer_cmds_msg)




    def drive_motor_conv(self):

        threshold = 0.05
        num_motors = 6
        

        if self.joystick_msg.axes[RIGHT_TRIGGER] > threshold:

            value = -(self.joystick_msg.axes[RIGHT_TRIGGER] - 1)

            self.drive_cmds_msg.data = [value] * self.drive_motors_num
            
        
        elif joystick_msg.axes[LEFT_TRIGGER] > threshold:

            value = (self.joystick_msg.axes[LEFT_TRIGGER] - 1)
            
            self.drive_cmds_msg.data = [value] * self.drive_motors_num

        else:

            self.drive_cmds_msg.data = [value] * 0.0
            


    def steer_motor_conv_unlocked(self):

        """
        Take steer comands from RS and LS and populate steer_cmds_msg

        Implment such that the wheels can't turn in oppistie directions 
        """
        threshold = 0.2

        if self.joystick_msg.axes[LEFT_STICK_X] > threshold or self.joystick_msg.axe[LEFT_STICK_X] < -threshold:

            self.steer_cmds_msg[0:2] = map_value(self.joystick_msg.axes[LEFT_STICK_X])
        

        if self.joystick_msg.axes[RIGHT_STICK_X] > threshold or self.joystick_msg.axe[RIGHT_STICK_X] < -threshold:

            self.steer_cmds_msg[2:4] = map_value(self.joystick_msg.axes[RIGHT_STICK_X])

        

    def steer_motor_conv_locked(self):

        """
        Take steer commands from only LS ignore RS and populate steer_cmds_msg
        """

        threshold = 0.2

        if self.joystick_msg.axes[LEFT_STICK_X] > threshold or self.joystick_msg.axe[LEFT_STICK_X] < -threshold:

            self.steer_cmds_msg[0:2] = map_value(self.joystick_msg.axes[LEFT_STICK_X])
            self.steer_cmds_msg[2:4] = -(map_value(self.joystick_msg.axes[RIGHT_STICK_X]))

        

    def skid_steer_mode(self):

        skid_steer_speed = 0.4

        if self.joystick_msg.buttons[LEFT_BUMPER]:

            self.drive_cmds_msg.data[0:3] = [-(skid_steer_speed)]*3
            self.drive_cmds_msg.data[3:6] = [skid_steer_speed]*3

        elif self.joystick_msg.buttons[RIGHT_BUMPER]:

            self.drive_cmds_msg.data[0:3] = [skid_steer_speed]*3
            self.drive_cmds_msg.data[3:6] = [-(skid_steer_speed)]*3



    def steer_toggle(self):

    current_time = time.time()
    if current_time - self.last_toggle_time > self.toggle_debounce_time:
            self.steer_lock_state = not self.steer_lock_state
            self.last_toggle_time = current_time

    def map_value(value):
        
        return (value + 1) * (self.steer_left_limit - self.steer_right_limit) / (2) + self.steer_right_limit
    

def main(args=None):
    rclpy.init(args=args)

   
    SPEAR_Rover_Node = RoverControl()

    rclpy.spin(SPEAR_Rover_Node)

    SPEAR_Rover_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()