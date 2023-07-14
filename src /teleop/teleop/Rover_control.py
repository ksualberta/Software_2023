import rclpy
from rclpy.node import Node
from std_msgs import Float32MultiArray
from sensor_msgs import JointState
from rclpy.qos import QoSProfile

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

class RoverControl(Node):

    def __init__(self):
        super().__init__('rover_control')

        #Publsiher drive and steer commands
        self.drive_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = drive_topic, qos_profile = QoSProfile(depth=10))
        self.steer_cmds = self.create_publisher(msg_type = Float32MultiArray, topic = steer_topic, qos_profile = QoSProfile(depth=10))

        #Subscription to Joy commands         
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = joy_topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)

        timer = 0.2

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.rover_main_control)


    def JoystickMsg(self, msg):
        
        self.joystick_msg = msg
    
    def rover_main_control(self):
        self.drive_cmds_msg = Float32MultiArray()
        self.steer_cmds_msg = Float32MultiArray()


        pass

    def drive_motor_conv(self):


        #Take in RT and LT input. Convert that into floating point values and
        #populate drive_cmds_msg

        pass

    def steer_motor_conv_unlocked(self):

        """
        Take steer comands from RS and LS and populate steer_cmds_msg

        Implment such that the wheels can't turn in oppistie directions 
        """

        pass

    def steer_motor_conv_locked(self):

        """
        Take steer commands from only LS ignore RS and populate steer_cmds_msg
        """

        pass

    def skid_steer_mode(self):

        """
        If DPAD clicked bypass drive_motor_cmds and steer_motor_conv
        send 0 on steer_cmds_msg and send oppisited cmds to drive_cmds_msg
        Implment Deboucing mechanism 
        
        """

        pass

    def skid_toggle(self):

        """
        Toggle between a true and value switch 

        Use debouncing method eg.
        import time

class RoverControl(Node):
    def __init__(self):
        ...
        self.skid_mode_enabled = False
        self.last_toggle_time = time.time()
        self.toggle_debounce_time = 0.5  # Adjust this to change how long the user has to wait between toggles

    def skid_toggle(self):
        current_time = time.time()
        if current_time - self.last_toggle_time > self.toggle_debounce_time:
            self.skid_mode_enabled = not self.skid_mode_enabled
            self.last_toggle_time = current_time

        """

        pass