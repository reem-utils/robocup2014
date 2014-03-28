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
from util_states.sleeper import Sleeper

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random


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
            
            # Go to the init door
            smach.StateMachine.add(
                'go_door_in',
                nav_to_poi('door_init'),
                transitions={'succeeded': 'enter_door_init', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Cross init door 
            smach.StateMachine.add(
                'enter_door_init',
                EnterRoomSM('door_exit'),
                transitions={'succeeded': 'go_to_intermediate_state', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
          
            # Go to center point
            smach.StateMachine.add(
                'go_to_intermediate_state',
                nav_to_poi('intermediate'),
                transitions={'succeeded': 'wait_time', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            
            # Test of robots, here we didnt know what will we have to do
            smach.StateMachine.add(
                 'wait_time',
                 Sleeper(3),
                 transitions={'succeeded': 'go_out', 
                 'aborted': 'aborted'})

            # Go to the exit door
            smach.StateMachine.add(
                'go_out',
                nav_to_poi('exit_init'),
                transitions={'succeeded': 'enter_door_out', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # Cross exit door
            smach.StateMachine.add(
                'enter_door_out',
                EnterRoomSM('exit_exit'),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  


                 




