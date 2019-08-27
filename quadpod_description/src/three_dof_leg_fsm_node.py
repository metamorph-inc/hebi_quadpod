#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
# Name: three_dof_leg_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/24/2017
# Edit Date: 10/24/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# 3-dof leg with a z-axis hip joint, a x-axis knee joint,
# and a x-axis ankle joint
'''

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Float32, String
import time
import math
import sys

class MoveToTargetPos(State):
    def __init__(self, side,
                 hip_target_pos, knee_target_pos, ankle_target_pos,
                 hip_angle_topic, hip_cmd_topic,
                 knee_angle_topic, knee_cmd_topic,
                 ankle_angle_topic, ankle_cmd_topic,
                 pub_to_master_topic, success_trigger_msg,
                 ros_rate, state_name):
        State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'],
                             output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                          'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'])

        if (side == 'right'):
            self.hip_target_pos = hip_target_pos
            self.knee_target_pos = knee_target_pos
            self.ankle_target_pos = ankle_target_pos
        elif (side == 'left'):
            self.hip_target_pos = -hip_target_pos
            self.knee_target_pos = -knee_target_pos
            self.ankle_target_pos = -ankle_target_pos
        else:
            raise ValueError("Invalid argument: " + str(side) +
                             " ! Constructor requires 'left' or 'right' as arguments for the 'side' parameter")

        # state machine stuff
        self.hip_current_pos = None  # These get initialized in execute
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.hip_sub = rospy.Subscriber(hip_angle_topic, Float32, self.hip_callback)        # Subscribe to the hip joint's angle topic
        self.hip_pub = rospy.Publisher(hip_cmd_topic, Float32, queue_size=1)                # Publish commands to the hip joint's cmd topic
        self.knee_sub = rospy.Subscriber(knee_angle_topic, Float32, self.knee_callback)     # Subscribe to the knee joint's angle topic
        self.knee_pub = rospy.Publisher(knee_cmd_topic, Float32, queue_size=1)              # Publish commands to the knee joint's cmd topic
        self.ankle_sub = rospy.Subscriber(ankle_angle_topic, Float32, self.ankle_callback)  # Subscribe to the ankle joint's angle topic
        self.ankle_pub = rospy.Publisher(ankle_cmd_topic, Float32, queue_size=1)            # Publish commands to the ankle joint's cmd topic
        self.master_pub = rospy.Publisher(pub_to_master_topic, String, queue_size=100)      # Publish status messages to the master topic
        self.success_trigger_msg = success_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos

        timeout_counter = time.time()
        timeout_time = 20.0  # If the state doesn't achieve the target position within
                             # this time, it will return a 'failure' outcome.
        settle_tolerance = 0.08  # < 5 degrees
        settle_time = 0.5

        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False
        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        ankle_settle_start = None
        ankle_settle_countdown = False
        ankle_done = False

        # TODO: Make this communication more robust to compensate for TCP best-effort
        while True:
            if ((time.time() - timeout_counter) > timeout_time):
                self.update_userdata(userdata)
                self.clean_up()
                return 'failure'
            else:
                # Check hip joint success condition
                if (abs(self.hip_current_pos - self.hip_target_pos) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = time.time()
                    elif ((time.time() - hip_settle_start) > settle_time):
                        hip_done = True
                # Check knee joint success condition
                if (abs(self.knee_current_pos - self.knee_target_pos) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = time.time()
                    elif ((time.time() - knee_settle_start) > settle_time):
                        knee_done = True
                # Check ankle joint success condition
                if (abs(self.ankle_current_pos - self.ankle_target_pos) < settle_tolerance):
                    if not ankle_settle_countdown:
                        ankle_settle_countdown = True
                        ankle_settle_start = time.time()
                    elif ((time.time() - ankle_settle_start) > settle_time):
                        ankle_done = True
                if (hip_done and knee_done and ankle_done):
                    if self.success_trigger_msg is not None:
                        print("State: " + self.state_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                        self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                    self.clean_up()
                    self.update_userdata(userdata)
                    return 'success'  # You're done!
                else:
                    # TODO: Add a position/effort ramp/transition function
                    self.hip_pub.publish(self.hip_target_pos)
                    self.knee_pub.publish(self.knee_target_pos)
                    self.ankle_pub.publish(self.ankle_target_pos)
            self.rate.sleep()

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def ankle_callback(self, msg):
        if self.active_flag:
            self.ankle_current_pos = msg.data

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos
        userdata.hip_target_pos = self.hip_target_pos
        userdata.knee_target_pos = self.knee_target_pos
        userdata.ankle_target_pos = self.ankle_target_pos

    def clean_up(self):
        self.active_flag = False

class MoveAlongTargetPath(State):
    def __init__(self, side, max_rot_rate, settle_time,
                 hip_target_pos_list, knee_target_pos_list, ankle_target_pos_list,
                 hip_angle_topic, hip_cmd_topic,
                 knee_angle_topic, knee_cmd_topic,
                 ankle_angle_topic, ankle_cmd_topic,
                 pub_to_master_topic, success_trigger_msg,
                 ros_rate, state_name):
        State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'],
                             output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                          'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'])

        self.max_rot_rate = max_rot_rate  # rad/s
        self.settle_time = settle_time    # s
        if (side == 'right'):
            self.hip_target_pos_list = hip_target_pos_list
            self.knee_target_pos_list = knee_target_pos_list
            self.ankle_target_pos_list = ankle_target_pos_list
        elif (side == 'left'):
            self.hip_target_pos_list = [-angle if angle is not None else angle for angle in hip_target_pos_list ]
            self.knee_target_pos_list = [-angle if angle is not None else angle for angle in knee_target_pos_list]
            self.ankle_target_pos_list = [-angle if angle is not None else angle for angle in ankle_target_pos_list]
        else:
            raise ValueError("Invalid argument: " + str(side) +
                             " ! Constructor requires 'left' or 'right' as arguments for the 'side' parameter")

        # state machine stuff
        self.hip_current_pos = None  # These get initialized in execute
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.hip_sub = rospy.Subscriber(hip_angle_topic, Float32, self.hip_callback)        # Subscribe to the hip joint's angle topic
        self.hip_pub = rospy.Publisher(hip_cmd_topic, Float32, queue_size=1)                # Publish commands to the hip joint's cmd topic
        self.knee_sub = rospy.Subscriber(knee_angle_topic, Float32, self.knee_callback)     # Subscribe to the knee joint's angle topic
        self.knee_pub = rospy.Publisher(knee_cmd_topic, Float32, queue_size=1)              # Publish commands to the knee joint's cmd topic
        self.ankle_sub = rospy.Subscriber(ankle_angle_topic, Float32, self.ankle_callback)  # Subscribe to the ankle joint's angle topic
        self.ankle_pub = rospy.Publisher(ankle_cmd_topic, Float32, queue_size=1)            # Publish commands to the ankle joint's cmd topic
        self.master_pub = rospy.Publisher(pub_to_master_topic, String, queue_size=100)      # Publish status messages to the master topic
        self.success_trigger_msg = success_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos
        print("receiving userdata: ", self.hip_current_pos, self.knee_current_pos, self.ankle_current_pos)

        # postion ramp args
        t_prev = rospy.Time.now().to_sec()
        hip_pos_prev = self.hip_current_pos
        knee_pos_prev = self.knee_current_pos
        ankle_pos_prev = self.ankle_current_pos

        timeout_counter = rospy.Time.now().to_sec()
        timeout_time = 20.0  # If the state doesn't achieve the target position within
                             # this time, it will return a 'failure' outcome.
        settle_tolerance = 0.08  # < 5 degrees

        step = 0
        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False
        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        ankle_settle_start = None
        ankle_settle_countdown = False
        ankle_done = False
        # TODO: Make this communication more robust to compensate for TCP best-effort
        while True:
            hip_target = hip_pos_prev
            knee_target = knee_pos_prev
            ankle_target = ankle_pos_prev
            if self.hip_target_pos_list[step] is not None:
                hip_target = self.hip_target_pos_list[step]
            if self.knee_target_pos_list[step] is not None:
                knee_target = self.knee_target_pos_list[step]
            if self.ankle_target_pos_list[step] is not None:
                ankle_target = self.ankle_target_pos_list[step]
            if ((rospy.Time.now().to_sec() - timeout_counter) > timeout_time):
                self.update_userdata(userdata)
                self.clean_up()
                return 'failure'
            else:
                # Check hip joint success condition
                if (abs(self.hip_current_pos - hip_target) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - hip_settle_start) > self.settle_time):
                        hip_done = True
                # Check knee joint success condition
                if (abs(self.knee_current_pos - knee_target) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - knee_settle_start) > self.settle_time):
                        knee_done = True
                # Check ankle joint success condition
                if (abs(self.ankle_current_pos - ankle_target) < settle_tolerance):
                    if not ankle_settle_countdown:
                        ankle_settle_countdown = True
                        ankle_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - ankle_settle_start) > self.settle_time):
                        ankle_done = True
                if (hip_done and knee_done and ankle_done):
                    if (step+1 < len(self.hip_target_pos_list) or step+1 < len(self.knee_target_pos_list) or step+1 < len(self.ankle_target_pos_list)):
                        step = step + 1
                        hip_settle_start = None
                        hip_settle_countdown = False
                        hip_done = False
                        knee_settle_start = None
                        knee_settle_countdown = False
                        knee_done = False
                        ankle_settle_start = None
                        ankle_settle_countdown = False
                        ankle_done = False
                    else:
                        if self.success_trigger_msg is not None:
                            print("State: " + self.state_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                            self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                        self.clean_up()
                        self.update_userdata(userdata)
                        return 'success'  # You're done!
                else:
                    t_now = rospy.Time.now().to_sec()
                    self.hip_pub.publish(self.ramped_rot(hip_pos_prev, hip_target, t_prev, t_now, self.max_rot_rate))
                    self.knee_pub.publish(self.ramped_rot(knee_pos_prev, knee_target, t_prev, t_now, self.max_rot_rate))
                    self.ankle_pub.publish(self.ramped_rot(ankle_pos_prev, ankle_target, t_prev, t_now, self.max_rot_rate))
                    t_prev = t_now
                    hip_pos_prev = self.hip_current_pos
                    knee_pos_prev = self.knee_current_pos
                    ankle_pos_prev = self.ankle_current_pos
            self.rate.sleep()

    def ramped_rot(self, pos_prev, pos_target, t_prev, t_now, ramp_rate):
        # compute maximum rotation step
        step = ramp_rate * (t_now - t_prev)
        sign = 1.0 if (pos_target > pos_prev) else -1.0
        error = math.fabs(pos_target - pos_prev)
        if error < step:  # if we can get to target position within timestep, then we're done
            return pos_target
        else:
            return pos_prev + sign*step  # take a step toward the target position

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def ankle_callback(self, msg):
        if self.active_flag:
            self.ankle_current_pos = msg.data

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos
        userdata.hip_target_pos = self.hip_target_pos_list[-1]
        userdata.knee_target_pos = self.knee_target_pos_list[-1]
        userdata.ankle_target_pos = self.ankle_target_pos_list[-1]

    def clean_up(self):
        self.active_flag = False

class WaitForMasterCmd(State):
    def __init__(self,
                 hip_angle_topic, hip_cmd_topic,
                 knee_angle_topic, knee_cmd_topic,
                 ankle_angle_topic, ankle_cmd_topic,
                 sub_to_master_topic,
                 ros_rate, state_name,
                 *master_trigger_msgs):
        State.__init__(self, outcomes=list(master_trigger_msgs),
                       input_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos',
                                   'hip_target_pos', 'knee_target_pos', 'ankle_target_pos'],
                       output_keys=['hip_current_pos', 'knee_current_pos', 'ankle_current_pos'])
        # state machine stuff
        self.hip_target_pos = None  # These get initialized in execute
        self.knee_target_pos = None
        self.ankle_target_pos = None
        self.hip_current_pos = None
        self.knee_current_pos = None
        self.ankle_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.hip_sub = rospy.Subscriber(hip_angle_topic, Float32, self.hip_callback)           # Subscribe to the hip joint's angle topic
        self.hip_pub = rospy.Publisher(hip_cmd_topic, Float32, queue_size=1)                   # Publish commands to the hip joint's cmd topic
        self.knee_sub = rospy.Subscriber(knee_angle_topic, Float32, self.knee_callback)        # Subscribe to the knee joint's angle topic
        self.knee_pub = rospy.Publisher(knee_cmd_topic, Float32, queue_size=1)                 # Publish commands to the knee joint's cmd topic
        self.ankle_sub = rospy.Subscriber(ankle_angle_topic, Float32, self.ankle_callback)     # Subscribe to the ankle joint's angle topic
        self.ankle_pub = rospy.Publisher(ankle_cmd_topic, Float32, queue_size=1)               # Publish commands to the ankle joint's cmd topic
        self.master_sub = rospy.Subscriber(sub_to_master_topic, String, self.master_callback)  # Publish status messages to the master topic
        self.msg_from_master = None
        self.master_trigger_msgs = master_trigger_msgs
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name

    def execute(self, userdata):
        self.active_flag = True
        self.hip_current_pos = userdata.hip_current_pos  # values from previous state
        self.knee_current_pos = userdata.knee_current_pos
        self.ankle_current_pos = userdata.ankle_current_pos
        self.hip_target_pos = userdata.hip_target_pos
        self.knee_target_pos = userdata.knee_target_pos
        self.ankle_target_pos = userdata.ankle_target_pos

        # TODO: Make this communication more robust to compensate for TCP best-effort
        while True:
            if self.msg_from_master is not None:
                # process msg_from_master
                for trigger_msg in self.master_trigger_msgs:
                    if (self.msg_from_master == trigger_msg):
                        self.update_userdata(userdata)
                        self.clean_up()
                        return trigger_msg
                print("Unrecognized msg_from_master: " + self.msg_from_master)
                self.msg_from_master = None
            else:
                # TODO: Add a position/effort ramp/transition function
                self.hip_pub.publish(self.hip_target_pos)
                self.knee_pub.publish(self.knee_target_pos)
                self.ankle_pub.publish(self.ankle_target_pos)
            self.rate.sleep()

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def ankle_callback(self, msg):
        if self.active_flag:
            self.ankle_current_pos = msg.data

    def master_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ", Received trigger: " + msg.data + " from master")
            self.msg_from_master = msg.data

    def update_userdata(self, userdata):
        print("saving userdata: ", self.hip_current_pos, self.knee_current_pos, self.ankle_current_pos)
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.ankle_current_pos = self.ankle_current_pos

    def clean_up(self):
        self.active_flag = False
        self.msg_from_master = None

def main():
    # ROS stuff
    rospy.init_node('three_dof_leg_fsm_node', anonymous=True) # 'anonymous' prevents collisions/shutdown
                                                              # so we can launch multiple three_dof_leg_fsm_node.py nodes

    side = None
    hip_angle_topic = None
    hip_cmd_topic = None
    knee_angle_topic = None
    knee_cmd_topic = None
    ankle_angle_topic = None
    ankle_cmd_topic = None
    master_sub_topic = None
    master_pub_topic = None
    ros_rate = None
    if ((len(sys.argv[-1]) < 7 and len(sys.argv) < 11)
            or (len(sys.argv[-1]) >= 7 and sys.argv[-1][0:7] != "__log:=" and len(sys.argv) < 11)
            or (len(sys.argv[-1]) >= 7 and sys.argv[-1][0:7] == "__log:=" and len(sys.argv) < 13)):
        print("usage: three_dof_leg_fsm_node.py side hip_angle_topic hip_cmd_topic knee_angle_topic knee_cmd_topic"
              + " ankle_angle_topic ankle_cmd_topic master_sub_topic master_pub_topic ros_rate")
    else:
        side = sys.argv[1]
        hip_angle_topic = sys.argv[2]
        hip_cmd_topic = sys.argv[3]
        knee_angle_topic = sys.argv[4]
        knee_cmd_topic = sys.argv[5]
        ankle_angle_topic = sys.argv[6]
        ankle_cmd_topic = sys.argv[7]
        master_sub_topic = sys.argv[8]
        master_pub_topic = sys.argv[9]
        ros_rate = float(sys.argv[10])

    print(sys.argv)

    # Create leg state instances
    wait_start = WaitForMasterCmd(hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                  master_sub_topic, ros_rate, 'wait_start', 'stand_up')
    stand_up_start = MoveToTargetPos(side, 0, -1.1299, 0, hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                     master_pub_topic, None, ros_rate, 'stand_up_start')
    stand_up_1 = MoveToTargetPos(side, 0, -1.1299, 2.2599, hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                 master_pub_topic, None, ros_rate, 'stand_up_1')
    stand_up_2 = MoveToTargetPos(side, 0, 0, 1.8680, hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                 master_pub_topic, None, ros_rate, 'stand_up_2')
    stand_up_done = MoveToTargetPos(side, 0, 0.7854, 0.7854, hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'stand_up_done')
    stand_up_path = MoveAlongTargetPath(side, 6.23, 0.05,
                                        [0.0]*11,
                                        [-1.2094, -1.2094, -0.934, -0.6655, -0.4221, -0.205, -0.0078, 0.1782, 0.362, 0.5558, 0.7854],
                                        [0, 2.4189, 2.3921, 2.3159, 2.1995, 2.0512, 1.8758, 1.6734, 1.439, 1.1579, 0.7854],
                                        hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                        master_pub_topic, 'done', ros_rate, 'stand_up_path')
    wait_stand = WaitForMasterCmd(hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                  master_sub_topic, ros_rate, 'wait_stand', 'lift_north')  # add more possible commands here
    lift_north_path = MoveAlongTargetPath(side, 6.23, 0.05,
                                          [None, None, None, 0.0],
                                          [0.5558, 0.362, 0.1782, 0.1782],
                                          [1.1579, 1.439, 1.6734, 1.6734],
                                          hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                          master_pub_topic, 'done', ros_rate, 'lift_north_path')
    wait_orders = WaitForMasterCmd(hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                   master_sub_topic, ros_rate, 'wait_orders', 'lift_north', 'plant_northwest', 'plant_southwest', 'reset_path')  # add more possible commands here
    plant_northwest_path = MoveAlongTargetPath(side, 6.23, 0.05,
                                               [0.1529, 0.1529, 0.1529],
                                               [None, None, 1.0228],
                                               [None, 0.005, 0.005],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_northwest_path')
    plant_southwest_path = MoveAlongTargetPath(side, 6.23, 0.05,
                                               [0.2190, 0.2190, 0.2190, 0.2190],
                                               [None, None, 0.8753, 0.8753],
                                               [None, 2.6180, 2.6180, 1.0453],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_southwest_path')
    reset_path = MoveAlongTargetPath(side, 6.23, 0.05,
                                     [0.0],
                                     [0.7854],
                                     [0.7854],
                                     hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                     master_pub_topic, 'done', ros_rate, 'reset_path')

    #TODO: Add more leg state instances here!!!

    # Create a SMACH state machine
    three_dof_leg_fsm_node = StateMachine(outcomes=['success'])
    three_dof_leg_fsm_node.userdata.hip_current_pos = 0.0  # initial values}
    three_dof_leg_fsm_node.userdata.knee_current_pos = 0.0
    three_dof_leg_fsm_node.userdata.ankle_current_pos = 0.0
    three_dof_leg_fsm_node.userdata.hip_target_pos = 0.0
    three_dof_leg_fsm_node.userdata.knee_target_pos = 0.0
    three_dof_leg_fsm_node.userdata.ankle_target_pos = 0.0

    # Open the SMACH state machine
    with three_dof_leg_fsm_node:
        # Add states to the container
        StateMachine.add('WAIT_START', wait_start, transitions={'stand_up':'STAND_UP_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                              'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('STAND_UP_PATH', stand_up_path, transitions={'success':'WAIT_STAND', 'failure':'STAND_UP_PATH'},
                                                         remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        # Alternatively....
        #StateMachine.add('WAIT_START', wait_start, transitions={'stand_up':'STAND_UP_START'},
        #                                           remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
        #                                                      'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        #StateMachine.add('STAND_UP_START', stand_up_start, transitions={'success':'STAND_UP_1', 'failure':'STAND_UP_START'},
        #                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        #StateMachine.add('STAND_UP_1', stand_up_1, transitions={'success':'STAND_UP_2', 'failure':'STAND_UP_1'},
        #                                           remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        #StateMachine.add('STAND_UP_2', stand_up_2, transitions={'success':'STAND_UP_DONE', 'failure':'STAND_UP_2'},
        #                                           remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        #StateMachine.add('STAND_UP_DONE', stand_up_done, transitions={'success':'WAIT_STAND', 'failure':'STAND_UP_DONE'},
        #                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('WAIT_STAND', wait_stand, transitions={'lift_north':'LIFT_NORTH_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                              'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('LIFT_NORTH_PATH', lift_north_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_NORTH_PATH'},
                                                             remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('WAIT_ORDERS', wait_orders, transitions={'lift_north':'LIFT_NORTH_PATH', 'plant_northwest':'PLANT_NORTHWEST_PATH', 'plant_southwest': 'PLANT_SOUTHWEST_PATH', 'reset_path':'RESET_PATH'},
                                                     remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                                'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('PLANT_NORTHWEST_PATH', plant_northwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_SOUTHWEST_PATH', plant_southwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_SOUTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('RESET_PATH', reset_path, transitions={'success':'WAIT_ORDERS', 'failure':'RESET_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})


    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('three_dof_leg_fsm_node' + str(rospy.get_name()), three_dof_leg_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: three_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: three_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    outcome = three_dof_leg_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
