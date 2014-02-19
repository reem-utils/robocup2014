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
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX
        # userdata is generated in run time and it's actually remapped so we can't really see what's in it
        rospy.sleep(1)
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
            self.userdata.nav_to_coord_goal = [0.5, 0.5, 0.0]
            smach.StateMachine.add(
                'enter_room',
                DummyStateMachine(),
                transitions={'succeeded': 'move_to_intermediate_poi', 'aborted': 'aborted', 'preempted': 'succeeded'})
                

            smach.StateMachine.add(
                'move_to_intermediate_poi',
                nav_to_coord(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                

# 
#             smach.StateMachine.add(
#                 'wait_state',
#                 DummyStateMachine(),
#                 transitions={'succeeded': 'DummyStateMachine_succeeded', 'aborted': 'DummyStateMachine_aborted', 'preempted': 'succeeded'})
#                 
# 
#             smach.StateMachine.add(
#                 'move_to_exit',
#                 DummyStateMachine(),
#                 transitions={'succeeded': 'DummyStateMachine_succeeded', 'aborted': 'DummyStateMachine_aborted', 'preempted': 'succeeded'})
#                 




