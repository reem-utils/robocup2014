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
from manipulation_states.play_motion_sm import play_motion_sm
from navigation_states.go_poi_listen_word import Go_Poi_Listen_Word
from speech_states.listen_and_check_word import ListenWordSM_Concurrent
from hri_states.acknowledgment import acknowledgment
from util_states.timeout import TimeOut
from util_states.state_concurrence import ConcurrenceRobocup
from util_states.concurrence_with_time import ConcurrenceTime
from navigation_states.enter_room import EnterRoomSM

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

SENTENCE_STOP='wait here'
SENTENCE_GO='reem'

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
    
    NOTE: Is necessary check the time in each test and while we are going to the next test?
    
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            self.userdata.manip_motion_to_play = ''
            
            # Say Start basic Functionalities
            smach.StateMachine.add(
                 'say_start_basic_functionalities',
                 text_to_say("I'm ready to start Basic Functionalities"),
                 transitions={'succeeded': 'go_to_door', 'aborted': 'say_going_pick_place'}) # TODO before it was say_going_pick_place
            
            # Say Go Pick and Place
            smach.StateMachine.add(
                 'say_going_pick_place',
                 text_to_say("I'm going to the Pick and Place location"),
                 transitions={'succeeded': 'enter_start_door', 'aborted': 'enter_start_door'}) 
            
            # Cross start door and go to intermediate point 
            smach.StateMachine.add(
                'enter_start_door',
                EnterRoomSM('kitchen_counter'),
                transitions={'succeeded': 'pick_timer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Do pick and place + TimeOut
            STATES = [PickPlaceSM()]
            STATE_NAMES = ["do_pick_and_place"]
        
            smach.StateMachine.add(
                "pick_timer",
                ConcurrenceTime(states=STATES, state_name=STATE_NAMES, timeout=180),
                transitions={'succeeded': 'say_going_avoid', 
                             'aborted':'say_going_avoid', 
                             'time_ends': "timeout_pick_and_place"})  
# 
#             smach.StateMachine.add(
#                 'do_pick_and_place',
#                 PickPlaceSM(),
#                 transitions={'succeeded': 'play_motion_state_2', 'aborted': 'play_motion_state_2', 
#                 'preempted': 'preempted'})   
                        
            # Say TimeOut 
            smach.StateMachine.add(
                'timeout_pick_and_place',
                text_to_say("The time for Pick and Place is finish"),
                transitions={'succeeded': 'play_motion_state', 'aborted': 'play_motion_state', 
                'preempted': 'preempted'})
            
            # Home position just in case that we are in pick position
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home', skip_planning=True),
                transitions={'succeeded': 'say_going_avoid',
                             'preempted':'say_going_avoid', 
                             'aborted':'play_motion_state'})   
            
            # Say Go Avoid that
            smach.StateMachine.add(
                 'say_going_avoid',
                 text_to_say("I'm going to the Avoid that Area"),
                 transitions={'succeeded': 'go_avoid_that', 'aborted': 'go_avoid_that'}) 
           
            # Go to avoid that
            smach.StateMachine.add(
                'go_avoid_that',
                nav_to_poi('avoid_that'),
                transitions={'succeeded': 'wait_avoid_that', 'aborted': 'go_avoid_that', 
                'preempted': 'preempted'})
            
            # Wait for Go command
            smach.StateMachine.add(
                'wait_avoid_that',
                ListenWordSM_Concurrent(SENTENCE_GO),
                transitions={'succeeded': 'confirm_move_avoid_that', 'aborted': 'wait_avoid_that', 
                'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'confirm_move_avoid_that',
                acknowledgment("yes", "Okey, I move"),
                transitions={'succeeded': 'avoid_timer', 'aborted': 'say_going_avoid', 
                'preempted': 'preempted'}) 
            
            # Do avoid that + TimeOut
            STATES = [Avoid_That()]
            STATE_NAMES = ["do_avoid_that"]

            smach.StateMachine.add(
                "avoid_timer",
                ConcurrenceTime(states=STATES, state_name=STATE_NAMES, timeout=180),
                transitions={'succeeded': 'say_going_what_say', 
                             'aborted':'say_going_what_say', 
                             'time_ends': "timeout_avoid_that"})
            
            # Say TimeOut 
            smach.StateMachine.add(
                'timeout_avoid_that',
                text_to_say("The time for Avoid That is finish"),
                transitions={'succeeded': 'say_going_what_say', 'aborted': 'say_going_what_say', 
                'preempted': 'preempted'})
            
            # Do avoid that
#             smach.StateMachine.add(
#                 'do_avoid_that',
#                 Avoid_That(),
#                 transitions={'succeeded': 'say_going_what_say', 'aborted': 'say_going_what_say', 
#                 'preempted': 'preempted'}) 
            
            # Say Go What did you say 
            smach.StateMachine.add(
                 'say_going_what_say',
                 text_to_say("I'm ready to start the What did you say test"),
                 transitions={'succeeded': 'wait_what_did_you_say', 'aborted': 'wait_what_did_you_say'}) 
        
            # Wait for Go command
            smach.StateMachine.add(
                'wait_what_did_you_say',
                ListenWordSM_Concurrent(SENTENCE_GO),
                transitions={'succeeded': 'confirm_move_what_say', 'aborted': 'wait_what_did_you_say', 
                'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'confirm_move_what_say',
                acknowledgment("yes", "Okey, I go"),
                transitions={'succeeded': 'what_say_timer', 'aborted': 'what_say_timer', 
                'preempted': 'preempted'}) 
            
            # Do what did you say + TimeOut
            STATES = [WhatSaySM()]
            STATE_NAMES = ["do_what_did_you_say"]
            
            smach.StateMachine.add(
                "what_say_timer",
                ConcurrenceTime(states=STATES, state_name=STATE_NAMES, timeout=180),
                transitions={'succeeded': 'say_finish_basic_functionalities', 
                             'aborted':'say_finish_basic_functionalities', 
                             'time_ends': "timeout_what_say"})
            
            # Say TimeOut 
            smach.StateMachine.add(
                'timeout_what_say',
                text_to_say("The time for What did you say is finish"),
                transitions={'succeeded': 'go_to_door', 'aborted': 'go_to_door', 
                'preempted': 'preempted'})
            
            # Do what did you say
#             smach.StateMachine.add(
#                 'do_what_did_you_say',
#                 WhatSaySM(),
#                 transitions={'succeeded': 'say_finish_basic_functionalities', 'aborted': 'say_finish_basic_functionalities', 
#                 'preempted': 'preempted'}) 

            # Go to the exit door 
            smach.StateMachine.add(
                'go_to_door',
                nav_to_poi('door_exit'),
                transitions={'succeeded': 'go_to_exit', 'aborted': 'go_to_exit', 
                'preempted': 'preempted'})
            
            # Go to the exit door 
            smach.StateMachine.add(
                'go_to_exit',
                nav_to_poi('door_B'),
                transitions={'succeeded': 'say_finish_basic_functionalities', 'aborted': 'say_finish_basic_functionalities', 
                'preempted': 'preempted'})
            
            # Say Finish basic Functionalities
            smach.StateMachine.add(
                 'say_finish_basic_functionalities',
                 text_to_say("I finished Basic Functionalities"),
                 transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            

