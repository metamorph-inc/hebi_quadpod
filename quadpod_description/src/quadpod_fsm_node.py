#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: quadpod_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/31/2017
# Edit Date: 10/31/2017
#
# Description:
# Finite state machine controlling the position and movements of
# a quadpod
'''

import time
import math
import argparse

import rospy
from smach import State, StateMachine, Concurrence
import smach_ros
from std_msgs.msg import Float32, String

class MoveLeg(State):
    def __init__(self,
                pub_to_leg_topic, pub_to_leg_msg,
                sub_to_leg_topic, success_leg_msg, failure_leg_msg,
                ros_rate, state_name):
       State.__init__(self, outcomes=['success', 'failure'])
       # ROS stuff
       self.leg_pub = rospy.Publisher(pub_to_leg_topic, String, queue_size=1)
       self.pub_to_leg_msg = pub_to_leg_msg
       self.leg_sub = rospy.Subscriber(sub_to_leg_topic, String, self.leg_callback)
       self.success_leg_msg = success_leg_msg
       self.failure_leg_msg = failure_leg_msg
       self.rate = rospy.Rate(ros_rate)
       self.state_name = state_name
       self.active_flag = False
       self.msg_from_leg = None

    def execute(self, userdata):
        self.active_flag = True

        # Send a message down the chain
        if self.pub_to_leg_msg is not None:
            print("State: " + self.state_name + ", Sent pub_to_leg_msg: " + self.pub_to_leg_msg)
            self.leg_pub.publish(self.pub_to_leg_msg)

        if (self.success_leg_msg is None and self.failure_leg_msg is None):
            return 'success'
        else:  # wait for a response to come back up chain - could add timeout here
            while True:
                if self.msg_from_leg is not None:
                    if (self.msg_from_leg == self.success_leg_msg):
                        self.clean_up()
                        return 'success'
                    elif (self.msg_from_leg == self.failure_leg_msg):
                        self.clean_up()
                        return 'failure'
                    else:
                        print("Unrecognized msg_from_leg: " + self.msg_from_leg)
                    self.msg_from_leg = None
                self.rate.sleep()

    def leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ", Received msg_from_leg: " + msg.data)
            self.msg_from_leg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.msg_from_leg = None

def main():
    # ROS stuff
    rospy.init_node('quadpod_fsm_node', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('-rr', '--ros_rate', type=float, default=100.0, help="type=float, default=100.0, Description='rate at which ROS node publishes'")
    parser.add_argument('-ffl', '--from_front_left', type=str, required=True, help="Description='front left leg ROS topic to subscribe to'")
    parser.add_argument('-tfl', '--to_front_left', type=str, required=True, help="Description='front left leg ROS topic to publish to'")
    parser.add_argument('-ffr', '--from_front_right', type=str, required=True, help="Description='front right leg ROS topic to subscribe to'")
    parser.add_argument('-tfr', '--to_front_right', type=str, required=True, help="Description='front right leg ROS topic to publish to'")
    parser.add_argument('-frl', '--from_rear_left', type=str, required=True, help="Description='rear left leg ROS topic to subscribe to'")
    parser.add_argument('-trl', '--to_rear_left', type=str, required=True, help="Description='rear left leg ROS topic to publish to'")
    parser.add_argument('-frr', '--from_rear_right', type=str, required=True, help="Description='rear right leg ROS topic to subscribe to'")
    parser.add_argument('-trr', '--to_rear_right', type=str, required=True, help="Description='rear right leg ROS topic to publish to'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    args = parser.parse_args()

    ros_rate = args.ros_rate
    leg_to_quadpod_dict = {'frontleft':args.from_front_left, 'frontright':args.from_front_right,
                           'rearleft':args.from_rear_left, 'rearright':args.from_rear_right}
    quadpod_to_leg_dict = {'frontleft':args.to_front_left, 'frontright':args.to_front_right,
                           'rearleft':args.to_rear_left, 'rearright':args.to_rear_right}

    leg_name_list = ['frontleft', 'frontright', 'rearleft', 'rearright']

    # State instance dictionary
    pub_to_leg_msg_list = ['stand_up', 'reset',
                           'lift',
                           'lift_north', 'plant_north',
                           'plant_northwest', 'plant_northeast', 'plant_southwest', 'plant_southeast',
                           'push_north', 'push_south', 'push_east', 'push_west',
                           'pull_east', 'pull_west',
                           'push_east_from_push_south', 'push_west_from_push_south',
                           'push_north_from_push_west', 'push_north_from_push_east']
    state_instance_dict = {}
    for msg in pub_to_leg_msg_list:
        msg_dict = {}
        for leg_name in leg_name_list:
            msg_dict[leg_name] = MoveLeg(quadpod_to_leg_dict[leg_name], msg,
                                         leg_to_quadpod_dict[leg_name], 'done', None, 100, leg_name + ': ' + msg)
        state_instance_dict[msg] = msg_dict

    # Concurrent instances
    # stand_up
    stand_up_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                               outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                       'REARLEFT':'success', 'REARRIGHT':'success'}})
    with stand_up_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['stand_up']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['stand_up']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['stand_up']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['stand_up']['rearright'])

    # reset  #TODO: This may need a major rehaul
    reset_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                             outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                     'REARLEFT':'success', 'REARRIGHT':'success'}})
    with reset_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['reset']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['reset']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['reset']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['reset']['rearright'])

    # tilt_northwest
    tilt_northwest_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                     outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                             'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_northwest_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_south']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_east']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['push_west']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['push_north']['rearright'])

    # tilt_northeast
    tilt_northeast_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                     outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                             'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_northeast_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_west']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_south']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['push_north']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['push_east']['rearright'])

    # tilt_southwest
    tilt_southwest_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                     outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                             'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_southwest_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_east']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_north']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['push_south']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['push_west']['rearright'])

    # tilt_southeast
    tilt_southeast_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                     outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                             'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_southeast_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_north']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_west']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['push_east']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['push_south']['rearright'])

    # tilt_northeast_after_step_con
    tilt_northeast_after_step_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                                outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                                        'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_northeast_after_step_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_west_from_push_south']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_south']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['push_north_from_push_west']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['pull_east']['rearright'])

    # tilt_northwest_after_step_con
    tilt_northwest_after_step_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                                outcome_map={'success':{'FRONTLEFT':'success', 'FRONTRIGHT':'success',
                                                                        'REARLEFT':'success', 'REARRIGHT':'success'}})
    with tilt_northwest_after_step_con:
        Concurrence.add('FRONTLEFT', state_instance_dict['push_south']['frontleft'])
        Concurrence.add('FRONTRIGHT', state_instance_dict['push_east_from_push_south']['frontright'])
        Concurrence.add('REARLEFT', state_instance_dict['pull_west']['rearleft'])
        Concurrence.add('REARRIGHT', state_instance_dict['push_north_from_push_east']['rearright'])

    # Create & open the top-level SMACH state machine
    quadpod_fsm_node = StateMachine(outcomes=['success'])
    with quadpod_fsm_node:
        # Add instances to the top-level StateMachine
        # Start-up
        StateMachine.add('STAND_UP', stand_up_con, transitions={'failure':'STAND_UP', 'success':'TILT_NORTHWEST'})
        StateMachine.add('TILT_NORTHWEST', tilt_northwest_con, transitions={'failure':'TILT_NORTHWEST', 'success':'REARRIGHT_LIFT'})
        # Cycle A
        StateMachine.add('REARRIGHT_LIFT', state_instance_dict['lift']['rearright'], transitions={'failure':'REARRIGHT_LIFT', 'success':'REARRIGHT_PLANT_SOUTHWEST'})
        StateMachine.add('REARRIGHT_PLANT_SOUTHWEST', state_instance_dict['plant_southwest']['rearright'], transitions={'failure':'REARRIGHT_PLANT_SOUTHWEST', 'success':'FRONTRIGHT_LIFT'})
        StateMachine.add('FRONTRIGHT_LIFT', state_instance_dict['lift']['frontright'], transitions={'failure':'FRONTRIGHT_LIFT', 'success':'FRONTRIGHT_PLANT_NORTH'})
        StateMachine.add('FRONTRIGHT_PLANT_NORTH', state_instance_dict['plant_north']['frontright'], transitions={'failure':'FRONTRIGHT_PLANT_NORTH', 'success':'TILT_NORTHEAST_AFTER_STEP'})
        StateMachine.add('TILT_NORTHEAST_AFTER_STEP', tilt_northeast_after_step_con, transitions={'failure':'TILT_NORTHEAST_AFTER_STEP', 'success':'REARLEFT_LIFT'})
        # Cycle B
        StateMachine.add('REARLEFT_LIFT', state_instance_dict['lift']['rearleft'], transitions={'failure':'REARLEFT_LIFT', 'success':'REARLEFT_PLANT_SOUTHEAST'})
        StateMachine.add('REARLEFT_PLANT_SOUTHEAST', state_instance_dict['plant_southeast']['rearleft'], transitions={'failure':'REARLEFT_PLANT_SOUTHEAST', 'success':'FRONTLEFT_LIFT'})
        StateMachine.add('FRONTLEFT_LIFT', state_instance_dict['lift']['frontleft'], transitions={'failure':'FRONTLEFT_LIFT', 'success':'FRONTLEFT_PLANT_NORTH'})
        StateMachine.add('FRONTLEFT_PLANT_NORTH', state_instance_dict['plant_north']['frontleft'], transitions={'failure':'FRONTLEFT_PLANT_NORTH', 'success':'TILT_NORTHWEST_AFTER_STEP'})
        StateMachine.add('TILT_NORTHWEST_AFTER_STEP', tilt_northwest_after_step_con, transitions={'failure':'TILT_NORTHWEST_AFTER_STEP', 'success':'REARRIGHT_LIFT'})
        # Repeat

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('quadpod_fsm_node' + str(rospy.get_name()), quadpod_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: quadpod_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: quadpod_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    outcome = quadpod_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
