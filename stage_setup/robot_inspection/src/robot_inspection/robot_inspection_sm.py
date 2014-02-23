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
from navigation_states.enter_room import EnterRoomSM
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'

class enter_door_in(smach.State) :

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Entering door, Cris is doing"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'

class setGoIntermediatePoi(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) #todo: i have to delate de output_key

    def execute(self,userdata):
        userdata.nav_to_poi_name='intermediate'
        return 'succeeded'


class setExit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name'])

    def execute(self,userdata):
        userdata.nav_to_poi_name='exit'
        return 'succeeded'

class prepareDoorInit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='doorIn'
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
            self.userdata.nav_to_poi_name=''
            
            
            smach.StateMachine.add(
                'prepare_door',
                prepareDoorInit(),
                transitions={'succeeded': 'go_door_in', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            #just triing
            smach.StateMachine.add(
                'go_door_in',
                nav_to_poi(),
                transitions={'succeeded': 'enter_door_in', 'aborted': 'aborted', 
                'preempted': 'preempted'})    


            #croos door
            smach.StateMachine.add(
                'enter_door_in',
                EnterRoomSM(),
                transitions={'succeeded': 'setGoIntermediatePoi', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
          
            # go to center
            smach.StateMachine.add(
                'setGoIntermediatePoi',
                setGoIntermediatePoi(),
                transitions={'succeeded': 'go_to_intermediate_state', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'go_to_intermediate_state',
                nav_to_poi(),
                transitions={'succeeded': 'wait_time', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            
            # test of robots, here we didnt know what will we have to do
            smach.StateMachine.add(
                 'wait_time',
                 DummyStateMachine(),
                 transitions={'succeeded': 'setExit', 
                 'aborted': 'aborted', 
                 'preempted': 'succeeded'})

            smach.StateMachine.add(
                'setExit',
                setExit(),
                transitions={'succeeded': 'go_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})

            smach.StateMachine.add(
                'go_out',
                nav_to_poi(),
                transitions={'succeeded': 'enter_door_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            #Cross door
            smach.StateMachine.add(
                'enter_door_out',
                enter_door_in(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  


                 




