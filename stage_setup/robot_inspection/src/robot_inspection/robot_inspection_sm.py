#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: sampfeiffer
"""


import rospy
#import copy
import smach
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_goal']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(10)
        return 'succeeded'

class enter_door_in(smach.State) :

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Entering door, Cris is doing"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(10)
        return 'succeeded'


class RobotInspectionSM(smach.StateMachine):
    """
    Executes a SM that does not much. Transitions 10 times
    randomly transitioning to succeeded or to aborted.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
             #goto DoorIn
            self.userdata.nav_to_poi_goal = 'doorIn'
            smach.StateMachine.add(
                'go_to_door_in',
                nav_to_poi(),
                transitions={'succeeded': 'enter_door_in', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            #croos door
            smach.StateMachine.add(
                'enter_door_in',
                enter_door_in(),
                transitions={'succeeded': 'move_to_center_poi', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
            # go to center
            self.userdata.nav_to_poi_goal = 'centerRoom'
            smach.StateMachine.add(
                'move_to_center_poi',
                nav_to_poi(),
                transitions={'succeeded': 'center_inspection', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            # test of roboots
            smach.StateMachine.add(
                 'center_inspection',
                 DummyStateMachine(),
                 transitions={'succeeded': 'go_to_door_out', 
                 'aborted': 'aborted', 
                 'preempted': 'succeeded'})

            self.userdata.nav_to_poi_goal = 'doorOut'
            smach.StateMachine.add(
                'go_to_door_out',
                nav_to_poi(),
                transitions={'succeeded': 'enter_door_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            #Cross door
            smach.StateMachine.add(
                'enter_door_out',
                enter_door_in(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  


                 




