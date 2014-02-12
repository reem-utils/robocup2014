#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: sampfeiffer
"""


import rospy
#import copy
import smach
#from smach_ros import SimpleActionState, ServiceState  # <- you'll need this!
#import actionlib

#from std_msgs.msg import *
#from actionlib_msgs import *
#from actionlib_msgs.msg import GoalStatus

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], input_keys=['coming_from_outcome', 'number_of_transitions'], output_keys=['coming_from_outcome', 'number_of_transitions'])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX
        # userdata is generated in run time and it's actually remapped so we can't really see what's in it
        rospy.loginfo("We came from outcome: " + userdata.coming_from_outcome)
        rospy.sleep(1)
        userdata.number_of_transitions += 1
        if userdata.number_of_transitions >= 10:
            rospy.loginfo("Returning preempted!")
            return 'preempted'
            
        if random.randint(0,1):
            rospy.loginfo(OKGREEN + "Returning succeeded!" + ENDC)
            userdata.coming_from_outcome = 'succeeded'
            return 'succeeded'
        else:
            rospy.loginfo(FAIL +"Returning aborted!" + ENDC)
            userdata.coming_from_outcome = 'aborted'
            return 'aborted'



class HelloWorldStateMachine(smach.StateMachine):
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
            self.userdata.coming_from_outcome = 'initial_outcome'
            self.userdata.number_of_transitions = 0
            smach.StateMachine.add(
                'DummyStateMachine_succeeded',
                DummyStateMachine(),
                transitions={'succeeded': 'DummyStateMachine_succeeded', 'aborted': 'DummyStateMachine_aborted', 'preempted': 'succeeded'})
                
            smach.StateMachine.add(
                'DummyStateMachine_aborted',
                DummyStateMachine(),
                transitions={'succeeded': 'DummyStateMachine_succeeded', 'aborted': 'DummyStateMachine_aborted',  'preempted': 'aborted'})



