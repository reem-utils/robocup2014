#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

26 Feb 2014
"""

import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from pick_place_sm import PickPlaceSM
from avoid_that_sm import Avoid_That
from what_say_sm import WhatSaySM


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class BasicFunctionalitiesSM(smach.StateMachine):
    """
    Executes a SM that does the basic functionalities.
    The robot moves autonomously to each activities and do it.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    
    NOTE: Is necessary check the time in each test and while we are goint to the next test?
    
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # Go to pick and place
            smach.StateMachine.add(
                'go_pick_and_place',
                nav_to_poi('init_pick_and_place'),
                transitions={'succeeded': 'do_pick_and_place', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do pick and place
            smach.StateMachine.add(
                'do_pick_and_place',
                PickPlaceSM(),
                transitions={'succeeded': 'go_avoid_that', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
           
            # Go to avoid that
            smach.StateMachine.add(
                'go_avoid_that',
                nav_to_poi('init_avoid_that'),
                transitions={'succeeded': 'do_avoid_that', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do avoid that
            smach.StateMachine.add(
                'do_avoid_that',
                Avoid_That(),
                transitions={'succeeded': 'go_what_did_you_say', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Go to what did you say
            smach.StateMachine.add(
                'go_what_did_you_say',
                nav_to_poi('init_what_say'),
                transitions={'succeeded': 'do_what_did_you_say', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do what did you say
            smach.StateMachine.add(
                'do_what_did_you_say',
                WhatSaySM(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 


