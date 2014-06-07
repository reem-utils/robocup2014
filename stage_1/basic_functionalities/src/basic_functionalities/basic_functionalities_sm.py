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
from speech_states.say import text_to_say
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
            
            # Say Start basic Functionalities
            smach.StateMachine.add(
                 'say_start_basic_functionalities',
                 text_to_say("I'm ready to start Basic Functionalities"),
                 transitions={'succeeded': 'say_going_what_say', 'aborted': 'say_going_pick_place'}) 
            
            # Say Go Pick and Place
            smach.StateMachine.add(
                 'say_going_pick_place',
                 text_to_say("I'm going to the Pick and Place poi"),
                 transitions={'succeeded': 'go_pick_and_place', 'aborted': 'go_pick_and_place'}) 
            
            # Go to pick and place
            smach.StateMachine.add(
                'go_pick_and_place',
                nav_to_poi('init_pick_and_place'),
                transitions={'succeeded': 'do_pick_and_place', 'aborted': 'say_going_pick_place', 
                'preempted': 'preempted'})    

            # Do pick and place
            smach.StateMachine.add(
                'do_pick_and_place',
                PickPlaceSM(),
                transitions={'succeeded': 'say_going_avoid', 'aborted': 'say_going_avoid', 
                'preempted': 'preempted'}) 
           
            # Say Go Avoid that
            smach.StateMachine.add(
                 'say_going_avoid',
                 text_to_say("I'm going to the Avoid that Area"),
                 transitions={'succeeded': 'go_avoid_that', 'aborted': 'go_avoid_that'}) 
           
            # Go to avoid that
            smach.StateMachine.add(
                'go_avoid_that',
                nav_to_poi('init_avoid_that'),
                transitions={'succeeded': 'do_avoid_that', 'aborted': 'say_going_avoid', 
                'preempted': 'preempted'})    

            # Do avoid that
            smach.StateMachine.add(
                'do_avoid_that',
                Avoid_That(),
                transitions={'succeeded': 'succeeded', 'aborted': 'go_what_did_you_say', 
                'preempted': 'preempted'}) 
            
            # Say Go What did you say 
            smach.StateMachine.add(
                 'say_going_what_say',
                 text_to_say("I'm going to the What did you say Location"),
                 transitions={'succeeded': 'go_what_did_you_say', 'aborted': 'go_what_did_you_say'}) 
            
            # Go to what did you say
            smach.StateMachine.add(
                'go_what_did_you_say',
                nav_to_poi('init_what_say'),
                transitions={'succeeded': 'do_what_did_you_say', 'aborted': 'say_going_what_say', 
                'preempted': 'preempted'})    

            # Do what did you say
            smach.StateMachine.add(
                'do_what_did_you_say',
                WhatSaySM(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 


