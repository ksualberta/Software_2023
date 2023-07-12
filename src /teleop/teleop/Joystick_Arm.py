import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

Joy_Topic = '/SPEAR_Arm/Joy_Topic'
Twist_Topic = "/SPEAR_Arm/delta_twist_cmds"
Joint_Topic = "/SPEAR_Arm/delta_joint_cmds"
EEF_Frame_ID = "link_5"
BASE_FRAME_ID = "base_link"


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


Axis_Default = {

    "LEFT_TRIGGER" : 1.0,
    "RIGHT_TRIGGER" : 1.0,
}

class Arm_Control(Node):

    def __init__(self):
        super().__init__('Arm_Control')
       
        self.frame_to_publish_ = BASE_FRAME_ID

        print("Working")

        self.twist_pub = self.create_publisher(msg_type = TwistStamped, topic = Twist_Topic, qos_profile = QoSProfile(depth=10))
        self.joint_pub = self.create_publisher(msg_type = JointJog, topic = Joint_Topic, qos_profile = QoSProfile(depth=10))

        print("Working 2")
        self.joy_sub = self.create_subscription(msg_type = Joy, topic = Joy_Topic, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.JoystickMsg)
        
        print("Working 3")
        timer_period = 0.1

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.JoyMain)
        
        self.joystick_msg = Joy()


    def JoyMain(self):


        self.twist_msg = TwistStamped()
        self.joint_msg = JointJog()
        
        print("Working 6")
        print(self.joystick_msg)

        if(self.ConvertJoyToCommand()):

            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = "link_5"
            self.twist_pub.publish(self.twist_msg)


        else:
            
            self.joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_msg.header.frame_id = "link_5"
            self.joint_pub.publish(self.joint_msg)


    def ConvertJoyToCommand(self):

        if(self.joystick_msg.buttons[A] or self.joystick_msg.buttons[B] or self.joystick_msg.buttons[X] 
        or self.joystick_msg.buttons[Y] or self.joystick_msg.axes[D_PAD_X] 
        or self.joystick_msg.axes[D_PAD_Y]):


            #Shoulder Roll 
            self.joint_msg.joint_names.append("Shoulder Roll")
            self.joint_msg.velocities.append(self.joystick_msg.axes[D_PAD_X])
            
            #Shoulder Pitch 
            self.joint_msg.joint_names.append("Shoulder Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.axes[D_PAD_Y])

            
            self.joint_msg.joint_names.append("Elbow Roll")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[B] - self.joystick_msg.buttons[X])


            self.joint_msg.joint_names.append("Elbow Pitch")
            self.joint_msg.velocities.append(self.joystick_msg.buttons[Y] - self.joystick_msg.buttons[A])

            return False 

        self.twist_msg.twist.linear.z = self.joystick_msg.axes[RIGHT_STICK_Y]
        self.twist_msg.twist.linear.y = self.joystick_msg.axes[RIGHT_STICK_X]
        
        
        lin_x_right = -0.5 * (self.joystick_msg.axes[RIGHT_TRIGGER]- Axis_Default["RIGHT_TRIGGER"])
        lin_x_left = 0.5 * (self.joystick_msg.axes[LEFT_TRIGGER]- Axis_Default["LEFT_TRIGGER"])
        self.twist_msg.twist.linear.x = lin_x_right + lin_x_left

        self.twist_msg.twist.angular.y = self.joystick_msg.axes[LEFT_STICK_Y]
        self.twist_msg.twist.angular.x = self.joystick_msg.axes[LEFT_STICK_X]
        
        roll_postive = 1.0 * self.joystick_msg.buttons[RIGHT_BUMPER]
        roll_negative = (-1.0) * self.joystick_msg.buttons[LEFT_BUMPER]
        self.twist_msg.twist.angular.z = roll_negative + roll_postive

        return True

    
    def JoystickMsg(self, msg):
        print("Working 4")
        self.joystick_msg = msg
        print(msg)



def main(args=None):
    rclpy.init(args=args)

   
    SPEAR_Arm_Node = Arm_Control()

    rclpy.spin(SPEAR_Arm_Node)

    SPEAR_Arm_Node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()