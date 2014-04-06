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
DummyStateMachine
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

class prepare_ask_person_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name'],
                                output_keys=['standard_error', 'tts_text'])

    def execute(self, userdata):
        
        userdata.tts_text = "I can't see you " + userdata.name + ". Can you come to me, please?"
        
        return 'succeeded'
    
class prepare_coord_wave(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['gesture_detection'],
                                output_keys=['standard_error', 'nav_to_coord_goal'])

    def execute(self, userdata):
        
        userdata.nav_to_coord_goal.x = userdata.gesture_detection.position.x
        userdata.nav_to_coord_goal.y = userdata.gesture_detection.position.y
        userdata.nav_to_coord_goal.yaw  = userdata.gesture_detection.orientation.w
        
        return 'succeeded'

class prepare_coord_order(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['face'],
                                output_keys=['standard_error', 'nav_to_coord_goal'])

    def execute(self, userdata):
        
        new_pose = Pose()
        new_pose.position.x = userdata.face.position.x
        new_pose.position.y = userdata.face.position.y

        unit_vector = normalize_vector(userdata.nav_to_coord_goal)
        position_distance = vector_magnitude(new_pose.position)

        distance_des = 0.0
        if position_distance >= self.distanceToHuman: 
            distance_des = position_distance - self.distanceToHuman
        else:
            rospy.loginfo(" Person too close => not moving, just rotate")
 
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        
        userdata.nav_to_coord_goal = [new_pose.position.x, new_pose.position.y, alfa]
        
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
            self.userdata.loop_iterations = 0
            
            # Must we say something to start? "I'm ready" or something
            # Must we wait for the spoken order? 
            
            # We wait for open door and go inside
            smach.StateMachine.add(
                'wait_for_door',
                EnterRoomSM("party_room"),
                transitions={'succeeded': 'gesture_recognition', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Gesture recognition -> Is anyone waving?
            smach.StateMachine.add(
                'gesture_recognition',
                GestureRecognition('wave'),
                transitions={'succeeded': 'prepare_coord_wave', 'aborted': 'ask_for_person', 
                'preempted': 'preempted'}) 

            # Prepare the goal to the person that is waving
            smach.StateMachine.add(
                'prepare_coord_wave',
                prepare_coord_wave(),
                transitions={'succeeded': 'go_to_person_wave', 'aborted': 'aborted', 
                'preempted': 'preempted'})             
            
            # Go to the person -> we assume that gesture will return the position
            smach.StateMachine.add(
                'go_to_person_wave',
                nav_to_coord('/base_link'),
                transitions={'succeeded': 'learning_person', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Ask for person if it can see anyone
            smach.StateMachine.add(
                'ask_for_person',
                text_to_say("I can't see anyone. Can anyone come to me, please?"),
                transitions={'succeeded': 'learning_person', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Learn Person -> Ask name + Face Recognition
            smach.StateMachine.add(
                'learning_person',
                SaveFaceSM(),
                transitions={'succeeded': 'ask_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Ask for order
            smach.StateMachine.add(
                'ask_order',
                AskQuestionSM("What would you like to drink?"),
                transitions={'succeeded': 'go_to_storage', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Go to the storage room
            smach.StateMachine.add(
                'go_to_storage',
                nav_to_poi('storage_room'),
                transitions={'succeeded': 'search_food_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Search for object
            smach.StateMachine.add(
                'search_food_order',
                DummyStateMachine(),
                transitions={'succeeded': 'grasp_food_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Grasp Object
            smach.StateMachine.add(
                'grasp_food_order',
                DummyStateMachine(),
                transitions={'succeeded': 'go_to_party', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Go to the party room
            smach.StateMachine.add(
                'go_to_party',
                nav_to_poi('party_room'),
                transitions={'succeeded': 'search_for_person', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Search for person -> He could change his position
            smach.StateMachine.add(
                'search_for_person',
                searching_person(),
                transitions={'succeeded': 'go_to_person', 'aborted': 'prepare_ask_for_person_back', 
                'preempted': 'preempted'}) 
            
            # Prepare the goal to the person that ask for the order
            smach.StateMachine.add(
                'prepare_coord_order',
                prepare_coord_order(),
                transitions={'succeeded': 'go_to_person_order', 'aborted': 'aborted', 
                'preempted': 'preempted'})             
            
            # Go to person
            smach.StateMachine.add(
                'go_to_person_order',
                nav_to_coord('/base_link'),
                transitions={'succeeded': 'deliver_drink', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Ask for person if it can see anyone
            smach.StateMachine.add(
                'prepare_ask_for_person_back',
                prepare_ask_person_back(),
                transitions={'succeeded': 'ask_for_person_back', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'ask_for_person_back',
                text_to_say(),
                transitions={'succeeded': 'deliver_drink', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Deliver Drink
            smach.StateMachine.add(
                'deliver_drink',
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
                'leaving_arena',
                nav_to_poi('leave_arena'),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            
            




