'''
Created on 12/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy
import math
from speech_states.say import text_to_say
from navigation_states.nav_to_coord import nav_to_coord
from face_states.recognize_face import recognize_face_concurrent
from geometry_msgs.msg import Pose
from util_states.math_utils import normalize_vector, vector_magnitude
from manipulation_states.give_object import give_object
from manipulation_states.play_motion_sm import play_motion_sm
from face_states.detect_faces import detect_face

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

class prepare_ask_person_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name'],
                                output_keys=['standard_error', 'tts_text'])

    def execute(self, userdata):
        
        userdata.tts_text = "I can't see you " + userdata.name + ". Can you come to me, please?"
        
        return 'succeeded'
 
 
class DeliverOrder(smach.StateMachine):
    """
    Executes a SM that execute one order in cocktail Party.
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['did_pick', 'object_name', 'name'],
                                    output_keys=[])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.did_pick = True

            # Say search for person
            smach.StateMachine.add(
                'say_search_person',
                text_to_say("I'm going to search the person who ordered me"),
                transitions={'succeeded': 'search_for_person', 'aborted': 'search_for_person', 
                'preempted': 'preempted'}) 
             
            # Search for person -> He could change his position
            smach.StateMachine.add(
                'search_for_person',
                #go_find_person("party_room"),
                recognize_face_concurrent(),
                transitions={'succeeded': 'say_found_person', 'aborted': 'prepare_ask_for_person_back', 
                'preempted': 'preempted'}) 
            
        # We recognize the person
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

        # Person recognition FAILS
            # Ask for person if it can see anyone
            smach.StateMachine.add(
                'prepare_ask_for_person_back',
                prepare_ask_person_back(),
                transitions={'succeeded': 'ask_for_person_back', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'ask_for_person_back',
                text_to_say(),
                transitions={'succeeded': 'wait_for_person_back', 'aborted': 'wait_for_person_back', 'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'wait_for_person_back',
                detect_face(),
                transitions={'succeeded': 'deliver_drink', 'aborted': 'aborted', 'preempted': 'preempted'}) 
            
        # Deliver part
            # Deliver Drink 
            smach.StateMachine.add(
                'deliver_drink',
                text_to_say("I'm going to deliver the drink"),
                transitions={'succeeded': 'check_place_give', 'aborted': 'check_place_give', 
                'preempted': 'preempted'}) 
            
            # Check if we pick or ask the drink
            smach.StateMachine.add(
                'check_place_give',
                check_place_give(),
                transitions={'succeeded':'pregrasp_state', 'aborted':'Give_Object', 'preempted':'check_place_give'})
            
            # Place if we pick the drink - Pre-grasp position
            smach.StateMachine.add(
                'pregrasp_state',
                play_motion_sm('pre_grasp', skip_planning=True),
                transitions={'succeeded': 'succeeded', 'preempted':'pregrasp_state', 
                             'aborted':'pregrasp_state'}) 
            
            # Give if we ask for drink 
            smach.StateMachine.add(
                'Give_Object',
                give_object(),
                transitions={'succeeded':'succeeded', 'aborted':'Give_Object', 'preempted':'Give_Object'})
