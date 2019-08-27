#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
# Name: three_dof_leg_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/24/2017
# Edit Date: 11/06/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# 3-dof leg with a z-axis hip joint, a x-axis knee joint,
# and a x-axis ankle joint
'''

import time
import math
import argparse

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Float32, String

class MoveToTargetPos(State):
    def __init__(self,
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

        self.hip_target_pos = hip_target_pos
        self.knee_target_pos = knee_target_pos
        self.ankle_target_pos = ankle_target_pos

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
        setimeout_time = 10.0  # If the state doesn't achieve the target position within
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
    def __init__(self, max_rot_rate, settle_time, timeout_time, best_effort,
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
        self.timeout_time = timeout_time  # s
        self.best_effort = best_effort
        self.hip_target_pos_list = hip_target_pos_list
        self.knee_target_pos_list = knee_target_pos_list
        self.ankle_target_pos_list = ankle_target_pos_list

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
            if ((rospy.Time.now().to_sec() - timeout_counter) > self.timeout_time):
                if self.best_effort:
                    if self.success_trigger_msg is not None:
                        print("State: " + self.state_name + ", Sent trigger: " + self.success_trigger_msg + " to master.")
                        self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'success'
                else:
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'false'
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
        if self.hip_target_pos_list[-1] is not None:
            userdata.hip_target_pos = self.hip_target_pos_list[-1]
        if self.knee_target_pos_list[-1] is not None:
            userdata.knee_target_pos = self.knee_target_pos_list[-1]
        if self.ankle_target_pos_list[-1] is not None:
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
    rospy.init_node('three_dof_leg_fsm_node', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('-rr', '--ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('-fh', '--from_hip', type=str, required=True, help="hip_angle_topic")
    parser.add_argument('-th', '--to_hip', type=str, required=True, help="hip_cmd_topic")
    parser.add_argument('-fk', '--from_knee', type=str, required=True, help="knee_angle_topic")
    parser.add_argument('-tk', '--to_knee', type=str, required=True, help="knee_cmd_topic")
    parser.add_argument('-fa', '--from_ankle', type=str, required=True, help="ankle_angle_topic")
    parser.add_argument('-ta', '--to_ankle', type=str, required=True, help="ankle_cmd_topic")
    parser.add_argument('-fm', '--from_master', type=str, required=True, help="master_angle_topic")
    parser.add_argument('-tm', '--to_master', type=str, required=True, help="master_cmd_topic")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    args = parser.parse_args()

    hip_angle_topic = args.from_hip
    hip_cmd_topic = args.to_hip
    knee_angle_topic = args.from_knee
    knee_cmd_topic = args.to_knee
    ankle_angle_topic = args.from_ankle
    ankle_cmd_topic = args.to_ankle
    master_sub_topic = args.from_master
    master_pub_topic = args.to_master
    ros_rate = args.ros_rate

    # Create leg state instances
    wait_start = WaitForMasterCmd(hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                  master_sub_topic, ros_rate, 'wait_start', 'stand_up')
    stand_up_path = MoveAlongTargetPath(6.23, 0.05, 10.0, True,
                                        [0.0]*11,
                                        [-1.2094, -1.2094, -0.934, -0.6655, -0.4221, -0.205, -0.0078, 0.1782, 0.362, 0.5558, 0.7854],
                                        [0, 2.4189, 2.3921, 2.3159, 2.1995, 2.0512, 1.8758, 1.6734, 1.439, 1.1579, 0.7854],
                                        hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                        master_pub_topic, 'done', ros_rate, 'stand_up_path')
    wait_orders = WaitForMasterCmd(hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                   master_sub_topic, ros_rate, 'wait_orders',
                                   'lift',
                                   'lift_north', 'plant_north',
                                   'plant_northwest', 'plant_northeast', 'plant_southwest', 'plant_southeast',
                                   'push_north', 'push_south', 'push_east', 'push_west',
                                   'pull_east', 'pull_west',
                                   'push_east_from_push_south', 'push_west_from_push_south',
                                   'push_north_from_push_east', 'push_north_from_push_west',
                                   'reset')
    reset_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0],
                                     [0.7854],
                                     [0.7854],
                                     hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                     master_pub_topic, 'done', ros_rate, 'reset_path')
    lift_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [None, None, None],
                                    [0.5558, 0.362, 0.1782],
                                    [1.1579, 1.439, 1.6734],
                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'lift_path')
    lift_north_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                          [None, None, None, 0.0],
                                          [0.5558, 0.362, 0.1782, 0.1782],
                                          [1.1579, 1.439, 1.6734, 1.6734],
                                          hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                          master_pub_topic, 'done', ros_rate, 'lift_north_path')
    plant_north_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                           [0.0]*3,
                                           [None, None, 1.0131],
                                           [None, 0.0194, 0.0194],
                                           hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                           master_pub_topic, 'done', ros_rate, 'plant_north_path')
    plant_northwest_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [0.1529, 0.1529, 0.1529],
                                               [None, None, 1.0228],
                                               [None, 0.005, 0.005],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_northwest_path')
    plant_northeast_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [-0.1529, -0.1529, -0.1529],
                                               [None, None, 1.0228],
                                               [None, 0.005, 0.005],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_northeast_path')
    plant_southwest_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [0.1939, 0.1939, 0.1939],
                                               [None, None, 0.7862],
                                               [None, 0.7515, 0.7515],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_southwest_path')
    plant_southeast_path = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                               [-0.1939, -0.1939, -0.1939],
                                               [None, None, 0.7862],
                                               [None, 0.7515, 0.7515],
                                               hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                               master_pub_topic, 'done', ros_rate, 'plant_southwest_path')
    push_north = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0]*10,
                                     [0.7854, 0.7864, 0.7897, 0.7955, 0.8044, 0.8171, 0.8351, 0.8607, 0.9007, 1.0131],
                                     [0.7854, 0.7464, 0.7035, 0.6561, 0.6031, 0.5431, 0.4733, 0.3886, 0.2759, 0.0194],
                                     hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                     master_pub_topic, 'done', ros_rate, 'push_north')
    push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                     [0.0]*10,
                                     [0.7854, 0.7863, 0.7891, 0.7934, 0.7993, 0.8065, 0.8151, 0.8249, 0.8359, 0.848],
                                     [0.7854, 0.8211, 0.8537, 0.8837, 0.9112, 0.9363, 0.9594, 0.9804, 0.9994, 1.0166],
                                     hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                     master_pub_topic, 'done', ros_rate, 'push_south')
    push_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [-0.0, -0.0218, -0.0436, -0.0654, -0.0871, -0.1087, -0.1302, -0.1516, -0.1728, -0.1939],
                                    [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7855, 0.7855, 0.7857, 0.7859, 0.7862],
                                    [0.7854, 0.785, 0.7838, 0.7817, 0.7789, 0.7752, 0.7706, 0.7651, 0.7588, 0.7515],
                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'push_east')
    push_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [0.0, 0.0218, 0.0436, 0.0654, 0.0871, 0.1087, 0.1302, 0.1516, 0.1728, 0.1939],
                                    [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7855, 0.7855, 0.7857, 0.7859, 0.7862],
                                    [0.7854, 0.785, 0.7838, 0.7817, 0.7789, 0.7752, 0.7706, 0.7651, 0.7588, 0.7515],
                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'push_west')
    pull_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [0.1939, 0.1728, 0.1516, 0.1302, 0.1087, 0.0871, 0.0654, 0.0436, 0.0218, 0.0],
                                    [0.7862, 0.7859, 0.7857, 0.7855, 0.7855, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854],
                                    [0.7515, 0.7588, 0.7651, 0.7706, 0.7752, 0.7789, 0.7817, 0.7838, 0.785, 0.7854],
                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'pull_east')
    pull_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                    [-0.1939, -0.1728, -0.1516, -0.1302, -0.1087, -0.0871, -0.0654, -0.0436, -0.0218, -0.0],
                                    [0.7862, 0.7859, 0.7857, 0.7855, 0.7855, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854],
                                    [0.7515, 0.7588, 0.7651, 0.7706, 0.7752, 0.7789, 0.7817, 0.7838, 0.785, 0.7854],
                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                    master_pub_topic, 'done', ros_rate, 'pull_east')
    push_east_from_push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [-0.0, -0.0271, -0.0543, -0.0813, -0.1082, -0.1349, -0.1615, -0.1878, -0.2139, -0.2397],
                                                    [0.848, 0.8478, 0.8473, 0.8465, 0.8453, 0.8438, 0.842, 0.8399, 0.8375, 0.8349],
                                                    [1.0166, 1.0164, 1.0157, 1.0146, 1.013, 1.011, 1.0085, 1.0055, 1.002, 0.9979],
                                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                                    master_pub_topic, 'done', ros_rate, 'push_east_from_push_south')
    push_west_from_push_south = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [0.0, 0.0271, 0.0543, 0.0813, 0.1082, 0.1349, 0.1615, 0.1878, 0.2139, 0.2397],
                                                    [0.848, 0.8478, 0.8473, 0.8465, 0.8453, 0.8438, 0.842, 0.8399, 0.8375, 0.8349],
                                                    [1.0166, 1.0164, 1.0157, 1.0146, 1.013, 1.011, 1.0085, 1.0055, 1.002, 0.9979],
                                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                                    master_pub_topic, 'done', ros_rate, 'push_west_from_push_south')
    push_north_from_push_west = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [0.1939, 0.1902, 0.1866, 0.1832, 0.1799, 0.1767, 0.1736, 0.1706, 0.1677, 0.1649],
                                                    [0.7862, 0.7887, 0.7933, 0.8002, 0.81, 0.8233, 0.8415, 0.8669, 0.906, 0.906],
                                                    [0.7515, 0.7136, 0.672, 0.6263, 0.5753, 0.5177, 0.4508, 0.37, 0.2624, 0.0084],
                                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                                    master_pub_topic, 'done', ros_rate, 'push_north_from_push_west')
    push_north_from_push_east = MoveAlongTargetPath(6.23, 0.05, 5.0, True,
                                                    [-0.1939, -0.1902, -0.1866, -0.1832, -0.1799, -0.1767, -0.1736, -0.1706, -0.1677, -0.1649],
                                                    [0.7862, 0.7887, 0.7933, 0.8002, 0.81, 0.8233, 0.8415, 0.8669, 0.906, 0.906],
                                                    [0.7515, 0.7136, 0.672, 0.6263, 0.5753, 0.5177, 0.4508, 0.37, 0.2624, 0.0084],
                                                    hip_angle_topic, hip_cmd_topic, knee_angle_topic, knee_cmd_topic, ankle_angle_topic, ankle_cmd_topic,
                                                    master_pub_topic, 'done', ros_rate, 'push_north_from_push_east')

    #TODO: Add more leg state instances here!!!

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
        StateMachine.add('STAND_UP_PATH', stand_up_path, transitions={'success':'WAIT_ORDERS', 'failure':'STAND_UP_PATH'},
                                                         remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('WAIT_ORDERS', wait_orders, transitions={'reset':'RESET_PATH',
                                                                  'lift':'LIFT_PATH',
                                                                  'lift_north':'LIFT_NORTH_PATH', 'plant_north':'PLANT_NORTH_PATH',
                                                                  'plant_northwest':'PLANT_NORTHWEST_PATH', 'plant_northeast':'PLANT_NORTHEAST_PATH',
                                                                  'plant_southwest': 'PLANT_SOUTHWEST_PATH', 'plant_southeast':'PLANT_SOUTHEAST_PATH',
                                                                  'push_north':'PUSH_NORTH', 'push_south':'PUSH_SOUTH', 'push_east':'PUSH_EAST', 'push_west':'PUSH_WEST',
                                                                  'pull_east':'PULL_EAST', 'pull_west':'PULL_WEST',
                                                                  'push_east_from_push_south':'PUSH_EAST_FROM_PUSH_SOUTH', 'push_west_from_push_south':'PUSH_WEST_FROM_PUSH_SOUTH',
                                                                  'push_north_from_push_east':'PUSH_NORTH_FROM_PUSH_EAST', 'push_north_from_push_west':'PUSH_NORTH_FROM_PUSH_WEST'},
                                                     remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos',
                                                                'hip_target_pos':'hip_target_pos', 'knee_target_pos':'knee_target_pos', 'ankle_target_pos':'ankle_target_pos'})
        StateMachine.add('RESET_PATH', reset_path, transitions={'success':'WAIT_ORDERS', 'failure':'RESET_PATH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('LIFT_PATH', lift_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_PATH'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('LIFT_NORTH_PATH', lift_north_path, transitions={'success':'WAIT_ORDERS', 'failure':'LIFT_NORTH_PATH'},
                                                             remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTH_PATH', plant_northwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTH_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTHWEST_PATH', plant_northwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_NORTHEAST_PATH', plant_northeast_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_NORTHEAST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_SOUTHWEST_PATH', plant_southwest_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_SOUTHWEST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PLANT_SOUTHEAST_PATH', plant_southeast_path, transitions={'success':'WAIT_ORDERS', 'failure':'PLANT_SOUTHEAST_PATH'},
                                                                       remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH', push_north, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_SOUTH', push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_SOUTH'},
                                                   remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_EAST', push_east, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_EAST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_WEST', push_west, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_WEST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PULL_EAST', pull_east, transitions={'success':'WAIT_ORDERS', 'failure':'PULL_EAST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PULL_WEST', pull_west, transitions={'success':'WAIT_ORDERS', 'failure':'PULL_WEST'},
                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_EAST_FROM_PUSH_SOUTH', push_east_from_push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_EAST_FROM_PUSH_SOUTH'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_WEST_FROM_PUSH_SOUTH', push_west_from_push_south, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_WEST_FROM_PUSH_SOUTH'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH_FROM_PUSH_WEST', push_north_from_push_west, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH_FROM_PUSH_WEST'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})
        StateMachine.add('PUSH_NORTH_FROM_PUSH_EAST', push_north_from_push_east, transitions={'success':'WAIT_ORDERS', 'failure':'PUSH_NORTH_FROM_PUSH_EAST'},
                                                                                 remapping={'hip_current_pos':'hip_current_pos', 'knee_current_pos':'knee_current_pos', 'ankle_current_pos':'ankle_current_pos'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('three_dof_leg_v2_fsm_node' + str(rospy.get_name()), three_dof_leg_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: three_dof_leg_v2_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: three_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    outcome = three_dof_leg_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
