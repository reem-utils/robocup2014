#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

Created on 22/03/2014
"""

import rospy
import smach
import math

from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from ask_order import AskOrder
from ask_all_orders import AskAllOrders
from execute_order import ExecuteOrder
from execute_all_order import ExecuteAllOrders

# Constants
NUMBER_OF_ORDERS = 3
NUMBER_OF_TRIES = 3
GRAMMAR_NAME = "robocup/drinks"

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class CocktailPartySM(smach.StateMachine):
    """
    Executes a SM that does the Cocktail Party.
    
    The robot goes inside a room, search for unknown persons that are waving and
    take the order. The robot goes to the storage room, take the correct food
    and return to the person and delivers the order.  
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 1
            self.userdata.try_iterations = 1
            self.userdata.gesture_name = ''
            self.userdata.object_name = []
            self.userdata.manip_time_to_play = 4
            self.userdata.did_pick = True
            self.userdata.grammar_name = GRAMMAR_NAME


            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home', skip_planning=True),
                transitions={'succeeded': 'init_cocktail',
                             'preempted':'init_cocktail', 
                             'aborted':'play_motion_state'})   
            
            smach.StateMachine.add(
                 'init_cocktail',
                 text_to_say("Ready for cocktail party"),
                 transitions={'succeeded': 'Ask_order', 'aborted': 'Ask_order'}) 
                  
            # We wait for open door and go inside
            smach.StateMachine.add(
                 'wait_for_door',
                 EnterRoomSM("party_room"),
                 transitions={'succeeded': 'Ask_order', 'aborted': 'aborted', 'preempted': 'preempted'}) 
               
            # Ask Order -> Wave + Learn Person + Order
            smach.StateMachine.add(
                'Ask_order',
                AskAllOrders(),
                transitions={'succeeded':'go_to_storage', 'aborted':'aborted'})   
            
#             # Go to the storage_room
#             smach.StateMachine.add(
#                 'go_to_storage',
#                 nav_to_poi("storage_room"),
#                 transitions={'succeeded': 'execute_order', 'aborted': 'go_to_storage', 
#                 'preempted': 'preempted'}) 
            
            # Execute the order 
            smach.StateMachine.add(
                'execute_order',
                ExecuteAllOrders(),
                transitions={'succeeded': 'say_leaving_arena', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Say leaving the arena 
            smach.StateMachine.add(
                'say_leaving_arena',
                text_to_say("I finished the cocktail party, I'm leaving the arena", wait=False),
                transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 
                'preempted': 'preempted'})             
            
            

            
            




