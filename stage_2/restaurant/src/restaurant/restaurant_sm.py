#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

Created on 6/04/2014
"""

import rospy
import smach
import math

from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.nav_to_poi_say import nav_to_poi_say

#from navigation_states.enter_room import EnterRoomSM
from speech_states.ask_question import AskQuestionSM
from speech_states.say import text_to_say
from restaurant_guide_phase import restaurantGuide
from restaurant_navigation import RestaurantNavigation
from restaurant_order_phase import RestaurantOrder

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("Dummy state just to change to other state")  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class RestaurantSM(smach.StateMachine):
    """
    Executes a SM that does the Restaurant.
    
    The robot is guided through the environment, showing 5 locations. 
    2 locations are for objects and 3 for delivery. 
    
    In the ordering location, the robot listen for a order. Then it goes to the 
    different objects locations and search for the object. If the object is found, it takes to 
    the deliver location. 
    
    The order contains 3 object and 2 location places. 

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
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.grammar_name="restaurant.gram"
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 0
            
            # Must we say something to start? "I'm ready" or something
            # Must we wait for the spoken order? 
                    
           # Guide phase
            smach.StateMachine.add(
                 'guide_phase',
                 restaurantGuide(),
                 transitions={'succeeded': 'start_restaurant', 'aborted': 'aborted', 
                 'preempted': 'preempted'}) 
            
            # Go to ordering location, i thinc it will not be necessary
            smach.StateMachine.add(
                'start_restaurant',
                nav_to_poi_say(tts="i am going to the ordering poi",poi_name='ordering'),
                transitions={'succeeded': 'ask_order', 'aborted': 'ask_order', 
                'preempted': 'preempted'}) 
                        
            # Ask for order
            smach.StateMachine.add(
                'ask_order',
                RestaurantOrder(),
                transitions={'succeeded': 'navigation_phase', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Navigation Phase
            smach.StateMachine.add(
                'navigation_phase',
                RestaurantNavigation(),
                transitions={'succeeded': 'leaving_ordering', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Leaving the arena  
            smach.StateMachine.add(
                'leaving_ordering',
                nav_to_poi('ordering'),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                 
