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
from util_states.math_utils import normalize_vector, vector_magnitude
from geometry_msgs.msg import Pose
from speech_states.parser_grammar import parserGrammar
from face_states.recognize_face import recognize_face_concurrent
from manipulation_states.play_motion_sm import play_motion_sm
from manipulation_states.ask_give_object_grasping import ask_give_object_grasping
from manipulation_states.give_object import give_object
from object_grasping_states.place_object_sm import place_object_sm
from geometry_msgs.msg import PoseStamped
from hri_states.recognize_object_and_pick import RecObjectAndPick
from ask_order import AskOrder
from ask_all_orders import AskAllOrders

# Constants
NUMBER_OF_ORDERS = 3
NUMBER_OF_TRIES = 3
GRAMMAR_NAME = "robocup/drinks"

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class dummy_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['object_position','pose_to_place'], 
            output_keys=['object_position','pose_to_place'])

    def execute(self, userdata):
        
        userdata.object_position = PoseStamped()
        userdata.object_position.header.frame_id = "base_link"
        userdata.object_position.pose.position.x = 1.0
        userdata.object_position.pose.position.z = 1.0
        userdata.object_position.pose.orientation.w = 1.0
        userdata.pose_to_place = PoseStamped()
        userdata.pose_to_place.header.frame_id = "base_link"
        userdata.pose_to_place.pose.position.x = 0.4
        userdata.pose_to_place.pose.position.z = 0.95
        userdata.pose_to_place.pose.orientation.w = 1.0
         
        rospy.sleep(5)
        return 'succeeded'


class prepare_ask_person_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name'],
                                output_keys=['standard_error', 'tts_text'])

    def execute(self, userdata):
        
        userdata.tts_text = "I can't see you " + userdata.name + ". Can you come to me, please?"
        
        return 'succeeded'
 
class change_did_pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=["did_pick"],
                                output_keys=["did_pick"])
        
    def execute(self, userdata):
    
        userdata.did_pick = False
        return 'succeeded'
         
class check_place_give(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=["did_pick"],
                                output_keys=[])
        
    def execute(self, userdata):
    
        if userdata.did_pick:
            return 'succeeded'
         
        return 'aborted'
    
class prepare_coord_order(smach.State):
    def __init__(self, distanceToHuman=0.3):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['face', 'nav_to_coord_goal'],
                                output_keys=['standard_error', 'nav_to_coord_goal'])
        self.distanceToHuman = distanceToHuman
        
    def execute(self, userdata):
        
        new_pose = Pose()
        new_pose.position.x = userdata.face.position.x
        new_pose.position.y = userdata.face.position.y

        unit_vector = normalize_vector(new_pose.position)
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
                                output_keys=['standard_error', 'loop_iterations', "did_pick"])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_ORDERS:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.did_pick = True
            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'
        
class checkRecognition(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['try_iterations', 'did_unk'],
                                output_keys=['standard_error', 'try_iterations', "did_unk"])

    def execute(self, userdata):
        
        if userdata.try_iterations == NUMBER_OF_TRIES:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.try_iterations = userdata.try_iterations + 1
            return 'succeeded'

class prepare_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['list_orders'],
                                output_keys=['object_name'])

    def execute(self, userdata):
        for item in userdata.list_orders:
            userdata.object_name.append(item[1])
        return 'succeeded'
    
class check_object_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['list_orders','object_detected_name'],
                                output_keys=['object_name'])

    def execute(self, userdata):
        for item in userdata.list_orders:
            if userdata.object_dectected_name == item[1]:
                userdata.name = item[0]
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
            self.userdata.loop_iterations = 1
            self.userdata.try_iterations = 1
            self.userdata.gesture_name = ''
            self.userdata.object_name = []
            self.userdata.manip_time_to_play = 4
            self.userdata.did_pick = True
            self.userdata.grammar_name = GRAMMAR_NAME

            smach.StateMachine.add(
                 'init_cocktail',
                 text_to_say("Ready for cocktail party"),
                 transitions={'succeeded': 'learning_person', 'aborted': 'wait_for_door'}) 
                  
            # We wait for open door and go inside
            smach.StateMachine.add(
                 'wait_for_door',
                 EnterRoomSM("party_room"),
                 transitions={'succeeded': 'Ask_order', 'aborted': 'aborted', 'preempted': 'preempted'}) 
               
            # Ask Order -> Wave + Learn Person + Order
            smach.StateMachine.add(
                'Ask_order',
                AskOrder(),
                transitions={'succeeded':'go_to_storage', 'aborted':'go_to_storage'})   
            
            # Go to the storage_room
            smach.StateMachine.add(
                'go_to_storage',
                nav_to_poi("storage_room"),
                transitions={'succeeded': 'recognize_object_and_pick', 'aborted': 'go_to_storage', 
                'preempted': 'preempted'}) 
            
            # Prepare info for recognize
            smach.StateMachine.add(
                'prepare_recognize',
                prepare_recognize(),
                transitions={'succeeded': 'recognize_object_and_pick', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # Recognize and pick object if found
            smach.StateMachine.add(
                'recognize_object_and_pick',
                RecObjectAndPick(),
                transitions={'succeeded': 'go_to_party', 
                             'fail_grasp':'recognize_object_and_pick',
                             'fail_recognize': 'try_again_recognition'})

            # Check which object had grasp
            smach.StateMachine.add(
                'check_object_grasp',
                check_object_grasp(),
                transitions={'succeeded': 'go_to_party', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # We don't recognized the object
            smach.StateMachine.add(
                'try_again_recognition',
                checkRecognition(),
                transitions={'succeeded': 'recognize_object_and_pick', 'aborted': 'recognize_object_and_pick', 
                'preempted': 'preempted', 'end':'say_fail_recognize'}) 
        
            # Say fail recognize objects
            smach.StateMachine.add(
                 'say_fail_recognize', 
                 text_to_say("I'm not able to recognized any object."),
                 transitions={'succeeded': 'Grasp_fail_Ask_Person', 'aborted': 'Grasp_fail_Ask_Person'})
            
            # Ask for grasp object
            smach.StateMachine.add(
                'Grasp_fail_Ask_Person',
                ask_give_object_grasping(),
                remapping={'object_to_grasp':'object_name'},
                transitions={'succeeded':'Rest_arm', 'aborted':'Rest_arm', 'preempted':'Rest_arm'})
            
            smach.StateMachine.add(
                 'Rest_arm',
                 play_motion_sm('rest_object_right'),
                 transitions={'succeeded':'change_did_pick', 'aborted':'change_did_pick', 'preempted':'change_did_pick'})
      
            smach.StateMachine.add(
                 'change_did_pick',
                 change_did_pick(),
                 transitions={'succeeded':'go_to_party', 'aborted':'go_to_party', 'preempted':'go_to_party'})
            
            # Go to the party room
            smach.StateMachine.add(
                'go_to_party',
                nav_to_poi('party_room'),
                transitions={'succeeded': 'say_search_person', 'aborted': 'go_to_party', 
                'preempted': 'preempted'}) 

            # Say search for person
            smach.StateMachine.add(
                'say_search_person',
                text_to_say("I'm going to search the person who ordered me"),
                transitions={'succeeded': 'search_for_person', 'aborted': 'search_for_person', 
                'preempted': 'preempted'}) 
             
            # Search for person -> He could change his position
            smach.StateMachine.add(
                'search_for_person',
                recognize_face_concurrent(),
                transitions={'succeeded': 'say_found_person', 'aborted': 'prepare_ask_for_person_back', 
                'preempted': 'preempted'}) 
            
            # Say found the person
            smach.StateMachine.add(
                'say_found_person',
                text_to_say("I found you!"),
                transitions={'succeeded': 'prepare_coord_order', 'aborted': 'aborted', 
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
                transitions={'succeeded': 'deliver_drink', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            
            # Deliver Drink 
            smach.StateMachine.add(
                'deliver_drink',
                text_to_say("I'm going to deliver the drink"),
                transitions={'succeeded': 'Give_Object', 'aborted': 'Give_Object', 
                'preempted': 'preempted'}) 
            
            # Check if we pick or ask the drink
            smach.StateMachine.add(
                'check_place_give',
                check_place_give(),
                transitions={'succeeded':'Place_Object', 'aborted':'Give_Object', 'preempted':'check_place_give'})
            
            # Place if we pick the drink
            smach.StateMachine.add(
                'Place_Object',
                place_object_sm(),
                transitions={'succeeded':'check_loop', 'aborted':'Place_Object', 'preempted':'Place_Object'})
            
            # Give if we ask for drink 
            smach.StateMachine.add(
                'Give_Object',
                give_object(),
                transitions={'succeeded':'check_loop', 'aborted':'Give_Object', 'preempted':'Give_Object'})
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'wave_recognition', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'say_leaving_arena'}) 
            
            # Say leaving the arena 
            smach.StateMachine.add(
                'say_leaving_arena',
                text_to_say("I finished the cocktail party, I'm leaving the arena", wait=False),
                transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 
                'preempted': 'preempted'})             
            
            

            
            




