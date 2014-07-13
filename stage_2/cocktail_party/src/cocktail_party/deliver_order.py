#! /usr/bin/env python
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
from geometry_msgs.msg import Pose, PointStamped, Point
from util_states.math_utils import normalize_vector, vector_magnitude
from manipulation_states.give_object import give_object
from manipulation_states.play_motion_sm import play_motion_sm
from face_states.detect_faces import detect_face
import tf
from hri_states.look_to_point import look_to_point

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
        
        rospy.loginfo("Getting a TransformListener...")
        tf_listener = tf.TransformListener()
        rospy.sleep(0.5) # we need to wait a moment so the tf listener inits

        ps = PointStamped()
        ps.header.frame_id = 'head_mount_xtion_rgb_optical_frame'
        ps.header.stamp = rospy.Time.now()
        ps.point = userdata.face.position

        # transform the pose of the face detection to base_link to compute on that frame
        transform_ok = False
        while not transform_ok: # this is ugly as is polling to TF... but works
            try:
                transformed_point = tf_listener.transformPoint("/base_link", ps)
                transform_ok = True
            except tf.ExtrapolationException, tf.LookupException:
                rospy.logwarn("Exception on transforming transformed_point... trying again.")
                ps.header.stamp = rospy.Time.now()
                rospy.sleep(0.3) # a real current time doesnt really work


        alfa = math.atan2(transformed_point.point.x, transformed_point.point.y)
        if transformed_point.point.x>self.distanceToHuman :
            transformed_point.point.x=transformed_point.point.x-self.distanceToHuman
        else :
            transformed_point.point.x=0
            transformed_point.point.y=0
        
        userdata.nav_to_coord_goal = [transformed_point.point.x, transformed_point.point.y, alfa]
        
        return 'succeeded'

class prepare_ask_person_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name_face'],
                                output_keys=['standard_error', 'tts_text'])

    def execute(self, userdata):
        
        userdata.tts_text = "I can't see you " + userdata.name_face + ". Can you come to me, please?"
        
        return 'succeeded'
 
class prepare_searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name_face'],
                                output_keys=['standard_error', 'tts_text','name'])

    def execute(self, userdata):
        
        userdata.tts_text = "I'm going to search " + userdata.name_face 
        userdata.name=userdata.name_face
        
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
                                    input_keys=['did_pick', 'object_name', 'name_face'],
                                    output_keys=[])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.did_pick = True


            smach.StateMachine.add(
                'look_to_point',
                look_to_point(direction="up",min_duration=1.0),
                transitions={'succeeded': 'prepare_say_search_person','preempted':'preempted', 'aborted':'aborted'})
            # Say search for person
            smach.StateMachine.add(
                'prepare_say_search_person',
                prepare_searching(),
                transitions={'succeeded': 'say_search_person', 'aborted': 'search_for_person', 
                'preempted': 'preempted'}) 
            
            # Say search for person
            smach.StateMachine.add(
                'say_search_person',
                text_to_say(),
                transitions={'succeeded': 'search_for_person', 'aborted': 'search_for_person', 
                'preempted': 'preempted'}) 
             
            # Search for person -> He could change his position
            smach.StateMachine.add(
                'search_for_person',
                #go_find_person("party_room"),
                recognize_face_concurrent(time_out=15),
                transitions={'succeeded': 'say_found_person', 'aborted': 'prepare_ask_for_person_back', 
                'preempted': 'preempted'}) 
            
        # We recognize the person
            # Say found the person
            smach.StateMachine.add(
                'say_found_person',
                text_to_say("I found you!",wait=False),
                transitions={'succeeded': 'prepare_coord_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Prepare the goal to the person that ask for the order
            smach.StateMachine.add(
                'prepare_coord_order',
                prepare_coord_order(),
                transitions={'succeeded': 'deliver_drink', 'aborted': 'aborted', 
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
                text_to_say("Ok, I'm going to deliver the drink, please take it"),
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
            
            
            
            
            
            
def main():
    rospy.init_node('cocktail_deliver')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        sm.userdata.did_pick=True
        sm.userdata.object_name="Cocacola"
        sm.userdata.name_face="hannah"

        smach.StateMachine.add(
            'cocktail_execute',
            DeliverOrder(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
#     sis = smach_ros.IntrospectionServer(
#         'basic_functionalities_introspection', sm, '/BF_ROOT')
#     sis.start()

    sm.execute()

    rospy.spin()
 #   sis.stop()


if __name__ == '__main__':
    main()
           
