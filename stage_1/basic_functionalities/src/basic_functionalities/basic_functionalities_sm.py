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

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

SENTENCE_STOP='wait here'
SENTENCE_GO='ok go'

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
            self.userdata.manip_motion_to_play = ''
            
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home'),
                transitions={'succeeded': 'say_start_basic_functionalities',
                             'preempted':'say_start_basic_functionalities', 
                             'aborted':'say_start_basic_functionalities'})   
            
            # Say Start basic Functionalities
            smach.StateMachine.add(
                 'say_start_basic_functionalities',
                 text_to_say("I'm ready to start Basic Functionalities"),
                 transitions={'succeeded': 'say_going_pick_place', 'aborted': 'say_going_pick_place'}) 
            
            # Say Go Pick and Place
            smach.StateMachine.add(
                 'say_going_pick_place',
                 text_to_say("I'm going to the Pick and Place location"),
                 transitions={'succeeded': 'go_pick_and_place', 'aborted': 'go_pick_and_place'}) 
            
            # Go to pick and place
            smach.StateMachine.add(
                'go_pick_and_place',
                Go_Poi_Listen_Word('init_pick_and_place', SENTENCE_STOP),
                #Go_Poi_Listen_Word('point_room_one', SENTENCE_STOP),
                #transitions={'succeeded': 'say_going_avoid', 'aborted': 'confirm_stop_pick_place', 
                #'preempted': 'preempted'})    
                transitions={'succeeded':'do_pick_and_place', 'aborted': 'confirm_stop_pick_place', 
                'preempted': 'preempted'})
                
            # The robot wait for "move" to move to the poi
            smach.StateMachine.add(
                'confirm_stop_pick_place',
                acknowledgment("yes", "Okey, I stop"),
                transitions={'succeeded': 'wait_pick_and_place', 'aborted': 'wait_pick_and_place', 
                'preempted': 'preempted'})   
            
            smach.StateMachine.add(
                'wait_pick_and_place',
                ListenWordSM_Concurrent(SENTENCE_GO),
                transitions={'succeeded': 'confirm_move_pick_place', 'aborted': 'wait_pick_and_place', 
                'preempted': 'preempted'})   
            
            smach.StateMachine.add(
                'confirm_move_pick_place',
                acknowledgment("yes", "Okey, I move"),
                transitions={'succeeded': 'say_going_pick_place', 'aborted': 'say_going_pick_place', 
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
                Go_Poi_Listen_Word('init_avoid_that', SENTENCE_STOP),
                #Go_Poi_Listen_Word('point_room_two', SENTENCE_STOP),
                #transitions={'succeeded': 'say_going_what_say', 'aborted': 'confirm_stop_avoid_that', 
                #'preempted': 'preempted'})    
                transitions={'succeeded': 'do_avoid_that', 'aborted': 'confirm_stop_avoid_that', 
                'preempted': 'preempted'})
                  
            # The robot wait for "move" to move to the poi
            smach.StateMachine.add(
                'confirm_stop_avoid_that',
                acknowledgment("yes", "Okey, I stop"),
                transitions={'succeeded': 'wait_avoid_that', 'aborted': 'wait_avoid_that', 
                'preempted': 'preempted'})  
                        
            smach.StateMachine.add(
                'wait_avoid_that',
                ListenWordSM_Concurrent(SENTENCE_GO),
                transitions={'succeeded': 'confirm_move_avoid_that', 'aborted': 'wait_avoid_that', 
                'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'confirm_move_avoid_that',
                acknowledgment("yes", "Okey, I move"),
                transitions={'succeeded': 'say_going_avoid', 'aborted': 'say_going_avoid', 
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
                Go_Poi_Listen_Word('init_what_say', SENTENCE_STOP),
                #Go_Poi_Listen_Word('point_room_three', SENTENCE_STOP),
                #transitions={'succeeded': 'say_finish_basic_functionalities', 'aborted': 'confirm_stop_what_say', 
                #'preempted': 'preempted'})    
                transitions={'succeeded': 'do_what_did_you_say', 'aborted': 'confirm_stop_what_say', 
                             'preempted': 'preempted'})   
         
            # The robot wait for "move" to move to the poi
            smach.StateMachine.add(
                'confirm_stop_what_say',
                acknowledgment("yes", "Okey, I stop"),
                transitions={'succeeded': 'wait_what_did_you_say', 'aborted': 'wait_what_did_you_say', 
                'preempted': 'preempted'})  
            
            smach.StateMachine.add(
                'wait_what_did_you_say',
                ListenWordSM_Concurrent(SENTENCE_GO),
                transitions={'succeeded': 'confirm_move_what_say', 'aborted': 'wait_what_did_you_say', 
                'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'confirm_move_what_say',
                acknowledgment("yes", "Okey, I move"),
                transitions={'succeeded': 'say_going_what_say', 'aborted': 'say_going_what_say', 
                'preempted': 'preempted'}) 
            
            # Do what did you say
            smach.StateMachine.add(
                'do_what_did_you_say',
                WhatSaySM(),
                transitions={'succeeded': 'say_finish_basic_functionalities', 'aborted': 'say_finish_basic_functionalities', 
                'preempted': 'preempted'}) 

            # Say Finish basic Functionalities
            smach.StateMachine.add(
                 'say_finish_basic_functionalities',
                 text_to_say("I finished Basic Functionalities"),
                 transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            

