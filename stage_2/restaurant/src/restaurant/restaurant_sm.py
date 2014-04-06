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
from navigation_states.enter_room import EnterRoomSM
from navigation_states.nav_to_coord import nav_to_coord
from speech_states.say import text_to_say
from speech_states.ask_question import AskQuestionSM
from face_states.ask_name_learn_face import SaveFaceSM
from face_states.searching_person import searching_person
from gesture_states.gesture_recognition import GestureRecognition 
from util_states.math_utils import normalize_vector, vector_magnitude
from geometry_msgs.msg import Pose

# Constants
NUMBER_OF_ORDERS = 3

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

class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['loop_iterations'],
                                output_keys=['standard_error', 'loop_iterations'])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_ORDERS:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.loop_iterations = userdata.loop_iterations + 1
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
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 0
            
            # Must we say something to start? "I'm ready" or something
            # Must we wait for the spoken order? 
            
            # Guide phase
            smach.StateMachine.add(
                'guide_phase',
                DummyStateMachine(),
                transitions={'succeeded': 'start_restaurant', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Go to ordering location
            smach.StateMachine.add(
                'start_restaurant',
                nav_to_poi('ordering'),
                transitions={'succeeded': 'ask_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                        
            # Ask for order
            smach.StateMachine.add(
                'ask_order',
                AskQuestionSM("What would you like to order?"),
                transitions={'succeeded': 'process_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Process order
            smach.StateMachine.add(
                'process_order',
                DummyStateMachine(),
                transitions={'succeeded': 'search_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Search for object - The robot goes to the different object locations
            smach.StateMachine.add(
                'search_object',
                DummyStateMachine(),
                transitions={'succeeded': 'grasp_food_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Grasp Object
            smach.StateMachine.add(
                'grasp_object',
                DummyStateMachine(),
                transitions={'succeeded': 'go_to_delivery', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Go to the delivery place
            smach.StateMachine.add(
                'go_to_delivery',
                nav_to_poi(),
                transitions={'succeeded': 'deliver_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                        
            # Deliver object
            smach.StateMachine.add(
                'deliver_object',
                DummyStateMachine(),
                transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'gesture_recognition', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'leaving_arena'}) 

            # Leaving the arena  
            smach.StateMachine.add(
                'leaving_ordering',
                nav_to_poi('ordering'),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                 
