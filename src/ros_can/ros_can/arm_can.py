import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
import can
from can import Message
import time
import struct

#Topic names

joint_traj_topic = "/Arm_Group_controller/joint_trajectory"
joint_states_topics = "/joint_states"

class Arm_2_Can(Node):
    def __init__(self):
        super().__init__('arm2can')

        joint_traj_topic = self.create_subscription(msg_type = JointTrajectory, topic = joint_states_topics, qos_profile = rclpy.qos.qos_profile_system_default, callback= self.joint_traj_msg)

        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=500000)

        timer_period = 0.1

        self.timer = self.create_timer(timer_period_sec = timer_period, callback = self.arm2can)

    
    def joint_traj_msg(self, msg):
        joint_names  = ["Shoulder Roll", " Shoulder Pitch", "Elbow Roll", "Elbow Pitch", "Wrist Roll", "Wrist Pitch"]
        self.joint_pos = []
        self.joint_vel = []
        for joint in joint_names 
            self.joint_pos.append(joint_position_map(msg, joint))
            self.joint_vel.append(joint_vel_map(msg, joint))

        

    
    def arm2can(self):
        pass


    def pos_2_float32(self, joint_pos):
        
        priority = 4  
        frame_id = 0x10   
            
        can_messages = []
          
        for i in range of (len(joint_pos)):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            position = joint_pos[i]

            position_bytes = struct.pack('Bf', i, position)




        pass

    def vel_2_float32(self):
        pass


    
    def joint_position_map(self, msg, joint_name):

        index = msg.index(joint_name)
        pos_val = msg.points.positions[index]

        return pos_val
        

    def joint_velocity_map(self, msg, joint_name):
        
        index = msg.index(joint_name)
        vel_val = msg.points.positions[index]
        
        return vel_val