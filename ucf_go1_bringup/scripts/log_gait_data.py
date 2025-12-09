#!/usr/bin/env python3
import rospy
import csv
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import WrenchStamped

class GaitDataLogger:
    def __init__(self):
        rospy.init_node('gait_data_logger')
        
        self.joint_states = None
        self.commanded_positions = None
        self.foot_forces = {
            'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0
        }
        
        # Subscribers
        rospy.Subscriber('/go1_gazebo/joint_states', JointState, self.joint_state_cb)
        rospy.Subscriber('/go1_gazebo/traj_controller/command', JointTrajectory, self.command_cb)
        
        # Correct foot contact topic names
        rospy.Subscriber('/visual/FL_foot_contact/the_force', WrenchStamped, self.fl_contact_cb)
        rospy.Subscriber('/visual/FR_foot_contact/the_force', WrenchStamped, self.fr_contact_cb)
        rospy.Subscriber('/visual/RL_foot_contact/the_force', WrenchStamped, self.rl_contact_cb)
        rospy.Subscriber('/visual/RR_foot_contact/the_force', WrenchStamped, self.rr_contact_cb)
        
        # CSV file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = f'/home/mathuzala/gait_data_{timestamp}.csv'
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        
        # Header
        header = ['time', 
                  'FL_hip_cmd', 'FL_thigh_cmd', 'FL_calf_cmd',
                  'FL_hip_act', 'FL_thigh_act', 'FL_calf_act',
                  'FR_hip_cmd', 'FR_thigh_cmd', 'FR_calf_cmd',
                  'FR_hip_act', 'FR_thigh_act', 'FR_calf_act',
                  'RL_hip_cmd', 'RL_thigh_cmd', 'RL_calf_cmd',
                  'RL_hip_act', 'RL_thigh_act', 'RL_calf_act',
                  'RR_hip_cmd', 'RR_thigh_cmd', 'RR_calf_cmd',
                  'RR_hip_act', 'RR_thigh_act', 'RR_calf_act',
                  'FL_force_z', 'FR_force_z', 'RL_force_z', 'RR_force_z',
                  'FL_stance', 'FR_stance', 'RL_stance', 'RR_stance']
        self.writer.writerow(header)
        
        self.start_time = rospy.Time.now()
        rospy.loginfo(f"Logging gait data to {self.filename}")
        
    def joint_state_cb(self, msg):
        self.joint_states = msg
        
    def command_cb(self, msg):
        if msg.points:
            self.commanded_positions = msg.points[0].positions
            
    def fl_contact_cb(self, msg):
        self.foot_forces['FL'] = msg.wrench.force.z
        
    def fr_contact_cb(self, msg):
        self.foot_forces['FR'] = msg.wrench.force.z
        
    def rl_contact_cb(self, msg):
        self.foot_forces['RL'] = msg.wrench.force.z
        
    def rr_contact_cb(self, msg):
        self.foot_forces['RR'] = msg.wrench.force.z
        
    def log_data(self):
        if self.joint_states is None or self.commanded_positions is None:
            return
            
        t = (rospy.Time.now() - self.start_time).to_sec()
        
        js = self.joint_states
        cmd = self.commanded_positions
        
        # Threshold for stance detection (N)
        STANCE_THRESHOLD = 5.0
        
        row = [t,
               # FL commanded (indices 3,4,5)
               cmd[3], cmd[4], cmd[5],
               # FL actual
               js.position[js.name.index('FL_hip_joint')] if 'FL_hip_joint' in js.name else 0,
               js.position[js.name.index('FL_thigh_joint')] if 'FL_thigh_joint' in js.name else 0,
               js.position[js.name.index('FL_calf_joint')] if 'FL_calf_joint' in js.name else 0,
               # FR commanded (indices 0,1,2)
               cmd[0], cmd[1], cmd[2],
               # FR actual
               js.position[js.name.index('FR_hip_joint')] if 'FR_hip_joint' in js.name else 0,
               js.position[js.name.index('FR_thigh_joint')] if 'FR_thigh_joint' in js.name else 0,
               js.position[js.name.index('FR_calf_joint')] if 'FR_calf_joint' in js.name else 0,
               # RL commanded (indices 9,10,11)
               cmd[9], cmd[10], cmd[11],
               # RL actual
               js.position[js.name.index('RL_hip_joint')] if 'RL_hip_joint' in js.name else 0,
               js.position[js.name.index('RL_thigh_joint')] if 'RL_thigh_joint' in js.name else 0,
               js.position[js.name.index('RL_calf_joint')] if 'RL_calf_joint' in js.name else 0,
               # RR commanded (indices 6,7,8)
               cmd[6], cmd[7], cmd[8],
               # RR actual
               js.position[js.name.index('RR_hip_joint')] if 'RR_hip_joint' in js.name else 0,
               js.position[js.name.index('RR_thigh_joint')] if 'RR_thigh_joint' in js.name else 0,
               js.position[js.name.index('RR_calf_joint')] if 'RR_calf_joint' in js.name else 0,
               # Foot forces (z-axis)
               self.foot_forces['FL'],
               self.foot_forces['FR'],
               self.foot_forces['RL'],
               self.foot_forces['RR'],
               # Stance detection (1=stance, 0=swing)
               1 if abs(self.foot_forces['FL']) > STANCE_THRESHOLD else 0,
               1 if abs(self.foot_forces['FR']) > STANCE_THRESHOLD else 0,
               1 if abs(self.foot_forces['RL']) > STANCE_THRESHOLD else 0,
               1 if abs(self.foot_forces['RR']) > STANCE_THRESHOLD else 0]
               
        self.writer.writerow(row)
        
    def run(self):
        rate = rospy.Rate(50)  # 50 Hz logging
        while not rospy.is_shutdown():
            self.log_data()
            rate.sleep()
        self.csvfile.close()
        rospy.loginfo(f"Data saved to {self.filename}")

if __name__ == '__main__':
    try:
        logger = GaitDataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
