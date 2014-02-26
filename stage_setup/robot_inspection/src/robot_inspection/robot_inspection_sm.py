#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldu
@email: roger.boldu@gmail.com
"""


import rospy
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

# Class that prepare the exit door init
class setExitInit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name'])

    def execute(self,userdata):
        userdata.nav_to_poi_name='exit_init'
        return 'succeeded'

# Class that prepare the exit door exit
class setExitExit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name'])

    def execute(self,userdata):
        userdata.nav_to_poi_name='exit_exit'
        return 'succeeded'

# Class that prepare the value need for nav_to_poi
class prepareDoorInit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='door_init'
        return 'succeeded'

# Class that prepare the value need for nav_to_poi
class prepareDoorExit(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='door_exit'
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
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # We prepare the information to go to the init door
            smach.StateMachine.add(
                'prepare_door_init',
                prepareDoorInit(),
                transitions={'succeeded': 'go_door_in', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to the init door
            smach.StateMachine.add(
                'go_door_in',
                nav_to_poi(),
                transitions={'succeeded': 'prepare_door_exit', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Cross init door
            smach.StateMachine.add(
                'prepare_door_exit',
                prepareDoorExit(),
                transitions={'succeeded': 'enter_door_init', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            smach.StateMachine.add(
                'enter_door_init',
                EnterRoomSM(),
                transitions={'succeeded': 'setGoIntermediatePoi', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
          
            # Go to center point
            smach.StateMachine.add(
                'setGoIntermediatePoi',
                setGoIntermediatePoi(),
                transitions={'succeeded': 'go_to_intermediate_state', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'go_to_intermediate_state',
                nav_to_poi(),
                transitions={'succeeded': 'wait_time', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            
            # Test of robots, here we didnt know what will we have to do
            smach.StateMachine.add(
                 'wait_time',
                 DummyStateMachine(),
                 transitions={'succeeded': 'set_exit_init', 
                 'aborted': 'aborted', 
                 'preempted': 'succeeded'})

            # Go to the exit door
            smach.StateMachine.add(
                'set_exit_init',
                setExitInit(),
                transitions={'succeeded': 'go_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})

            smach.StateMachine.add(
                'go_out',
                nav_to_poi(),
                transitions={'succeeded': 'set_exit_exit', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # Cross exit door
            smach.StateMachine.add(
                'set_exit_exit',
                setExitExit(),
                transitions={'succeeded': 'enter_door_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})

            smach.StateMachine.add(
                'enter_door_out',
                EnterRoomSM(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  


                 




