#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldu
@email: roger.boldu@gmail.com
"""


import rospy
import smach

from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from navigation_states.get_current_robot_pose import get_current_robot_pose
from speech_states.say import text_to_say
from util_states.sleeper import Sleeper
from util_states.state_concurrence import ConcurrenceRobocup
from manipulation_states.play_motion_sm import play_motion_sm
from geometry_msgs.msg import PoseWithCovarianceStamped 


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class save_robot_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['pose_current'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):
        
        return 'succeeded'
    
class set_robot_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['pose_current'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        self.initialpose_pub.publish(userdata.pose_current)
        return 'succeeded'
    
class RobotInspectionSM(smach.StateMachine):
    """
    Executes a SM that does the robot inspection.
    It moves the robot to the enter door, call enter_room,
    pass the inspection (now we have a dummy state that only waits 5 secs)
    and go to the exit door. We assume that the exit door will be closed.


    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys. userdata.word

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # Indicate that we are ready
            smach.StateMachine.add(
                'say_ready_inspection',
                text_to_say("I'm ready for Robot Inspection test"),
                transitions= {'succeeded':'enter_start_door', 'aborted':'aborted', 'preempted':'preempted'})
            
            # Cross start door and go to intermediate point 
            smach.StateMachine.add(
                'enter_start_door',
                EnterRoomSM('intermediate'),
                transitions={'succeeded': 'robot_presentation', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
          
            # Robot presentation: Little talk + wave gesture
            STATES = [text_to_say("Hi everybody! My name is REEM."), play_motion_sm("wave", 10)]
            STATE_NAMES = ["say_presentation", "salute_wave"]
            outcome_map = {'succeeded': {"say_presentation": 'succeeded', "salute_wave": 'succeeded'}}
        
            smach.StateMachine.add(
                "robot_presentation",
                ConcurrenceRobocup(states=STATES, state_names=STATE_NAMES, outcome_map=outcome_map),
                transitions={'succeeded': 'home_position', 'aborted': "aborted"})
            
            # Home position
            smach.StateMachine.add(
                'home_position',
                play_motion_sm('home', 10),
                transitions={'succeeded': 'get_actual_pos', 'aborted': 'aborted', 'preempted': 'succeeded'})
           
            # Calculate the actual position
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pose(),
                transitions={'succeeded': 'wait_time', 'aborted': 'aborted', 'preempted': 'succeeded'})

            # Save position
            smach.StateMachine.add(
                'save_robot_position',
                save_robot_position(),
                transitions={'succeeded': 'wait_time', 'aborted': 'aborted', 'preempted':'preempted'})
            
            # Test of robot 
            smach.StateMachine.add(
                 'wait_time',
                 Sleeper(20),
                 transitions={'succeeded': 'end_time_inspection', 'aborted': 'aborted'})

            # Indicate that we are ready
            smach.StateMachine.add(
                'end_time_inspection',
                text_to_say("Time finished"),
                transitions= {'succeeded':'set_robot_position', 'aborted':'aborted', 'preempted':'preempted'})
            
            # Set position
            smach.StateMachine.add(
                'set_robot_position',
                set_robot_position(),
                transitions={'succeeded': 'cross_door_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})
                        
            # Go to the exit door and cross exit door 
            # If the door is open change EnterRoom for nav_to_poi
            smach.StateMachine.add(
                'cross_door_out',
                nav_to_poi('exit_door'),
                transitions={'succeeded': 'succeeded', 'aborted': 'cross_door_out', 
                'preempted': 'preempted'})  


                 




