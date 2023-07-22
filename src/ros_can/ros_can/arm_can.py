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

"""
Actuator ID:
Shoulder Roll =
Shoulder Pitch =
Elbow Roll =
Elbow Pitch =
Wrist Roll =
Wrist Pitch  =
"""

joint_traj_topic = "/Arm_Group_controller/joint_trajectory"
joint_states_topics = "/joint_states"

class Arm_2_Can(Node):
    def __init__(self):
        super().__init__('arm2can')

        self.joint_state = [0.0,0.0,0.0,0.0,0.0,0.0]

        joint_state_msg = JointState()

        joint_state_msg.name = ["Shoulder Roll", " Shoulder Pitch", "Elbow Roll", "Elbow Pitch", "Wrist Roll", "Wrist Pitch"]
        joint_state_msg.position = [0.0,0.0,0.0,0.0,0.0,0.0]



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
        joint_pos_can = pos_2_float32(self.joint_pos)
        joint_vel_can = vel_2_float32(self.joint_vel)
        
        self.can_publsih(joint_pos_can)
        self.can_publish(joint_vel_can)

        recv_msg = can_recive


    def pos_2_float32(self, joint_pos):
        
        priority = 13 
        frame_id = 0x11   
            
        can_messages = []
          
        for i in range of (len(joint_pos)):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            position = joint_pos[i]
            position_bytes = struct.pack('Bf', i+12, position)

        return can_messages
        pass

    def vel_2_float32(self):

        priority = 14 
        frame_id = 0x10   
            
        can_messages = []
          
        for i in range of (len(joint_pos)):
            arbitration_id = (priority << 24) | (frame_id << 8) | self.node_id
            position = joint_pos[i]

            position_bytes = struct.pack('Bf', i+20, position)

        return can_messages


        

    def can_publish(self, can_messages):

        
        try:
            for msg in can_messages:
                self.bus.send(msg)

        except TypeError:
            self.get_logger().error("CRTICAL: Message not being publised, type error")

        except Exception as e:
            self.get_logger().error(f"CRITICAL: Unexpected error in steer_float32_can: {e}")

    
    def can_recive(self):

        msg = self.bus.recv()

        return msg



    
    def float32_2_pos(self, can_msg):

        arbitration_id = can_msg.arbitration_id
        frame_id = self.get_frame_id(arbitration_id)
    
        if frame_id = 0x21:

            can_data = can_msg.data 
            actuator_id, position = struct.unpack('Bf', can_data)
            if actuator_id in range():#range of actuator ID
                self.joint_state[actuator_id - 12] = position



    def get_frame_id(arbitration_id):
        return (arbitration_id >> 8) & 0xFFFF

    
    def joint_position_map(self, msg, joint_name):

        index = msg.index(joint_name)
        pos_val = msg.points.positions[index]

        return pos_val
        

    def joint_velocity_map(self, msg, joint_name):
        
        index = msg.index(joint_name)
        vel_val = msg.points.positions[index]
        
        return vel_val

    def Joint_State_Update(self):
