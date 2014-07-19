#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

Created on 16/04/2014
"""

import rospy
import smach
import math

from navigation_states.nav_to_poi import nav_to_poi
#from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from smach.user_data import Remapper
from object_grasping_states.object_detection_and_grasping import object_detection_and_grasping_sm
from object_grasping_states.place_object_sm import place_object_sm
from geometry_msgs.msg import PoseStamped
from hri_states.recognize_object_and_pick import RecObjectAndPick
from object_grasping_states.pick_object_sm import pick_object_sm

# Constants
NUMBER_OF_ORDERS = 3
TIME_OUT_GRASP=4

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class prepare_delivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['delivery_loc'], 
            output_keys=['nav_to_poi_name'])
       

    def execute(self, userdata):
        userdata.nav_to_poi_name = userdata.delivery_loc
        return 'succeeded'

class DummyStateMachine(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])
        self.text = text

    def execute(self, userdata):
        rospy.loginfo(self.text)
        rospy.sleep(3)
        return 'succeeded'

class decide_next_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['object_array', 'object_index', 'object_name', 'object_loc', 'delivery_loc'],
                                output_keys=['standard_error', 'object_name',
                                              'object_loc', 'delivery_loc','tts_text'])
        

    def execute(self, userdata):
        # We obtain the next element of the array
        
        userdata.object_name = userdata.object_array[userdata.object_index][0]
        userdata.object_loc = userdata.object_array[userdata.object_index][1]
        userdata.delivery_loc = userdata.object_array[userdata.object_index][2]
        
        userdata.object_name="Pringles" # TODO: delete this line
        rospy.loginfo("Object_name: " + userdata.object_name)
        rospy.loginfo("Object_loc: " + userdata.object_loc)
        rospy.loginfo("delivery_loc " + userdata.delivery_loc)
        
        userdata.tts_text= " i am going to "+userdata.object_loc +" and i will grasp  :" + userdata.object_name + "and bring it to " + userdata.delivery_loc
        
        return 'succeeded'

class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['object_index'],
                                output_keys=['standard_error', 'object_index'])

    def execute(self, userdata):
        
        if userdata.object_index == NUMBER_OF_ORDERS - 1:
            return 'end'
        else:
            rospy.loginfo(userdata.object_index)
            userdata.standard_error='OK'
            userdata.object_index = userdata.object_index + 1
            return 'succeeded'

class RestaurantNavigation(smach.StateMachine):
    """
    Executes a SM that does the Restaurant Navigation.

    Given an array with the information {objectName, objectLocation, deliveryLocation}
    the robot goes to the objectLocation, take it and deliver to the location. 
    It does the same while the array has objects.  
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input keys:
        object_array: Array with the information of the order
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['object_array'])
        
        pose_place = PoseStamped()
        pose_place.header.frame_id = '/base_link'
        pose_place.pose.position.x = 0.0
        pose_place.pose.position.y = 0.0
        pose_place.pose.position.z = 1.0
        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.object_name = ''
            self.userdata.object_index = 0
            self.userdata.object_name=None
            self.userdata.time_out_grasp=TIME_OUT_GRASP

            # Process order
            smach.StateMachine.add(
                'decide_next_object',
                decide_next_object(),
                remapping={"object_loc": "nav_to_poi_name"},
                transitions={'succeeded': 'say_im_going', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                        # Deliver object
            smach.StateMachine.add(
                'say_im_going',
                text_to_say(),
                transitions={'succeeded': 'go_to_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Go to poi where the object can stay
            smach.StateMachine.add(
                'go_to_object',
                nav_to_poi(),
                transitions={'succeeded': 'say_grasp_object', 'aborted': 'go_to_object'})
                        

            # Grasp Object
            smach.StateMachine.add(
                'say_grasp_object',
                text_to_say("Grasping Object"),
                transitions={'succeeded': 'grasp_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
#             smach.StateMachine.add(
#                 'Search_and_grasp_Object',
#                 object_detection_and_grasping_sm(),
#                 transitions={'succeeded':'prepare_delivery_goal', 
#                              'fail_object_detection':'say_object_not_find', 
#                              'fail_object_grasping':'say_not_grasp',
#                              'aborted':'aborted',
#                              'preempted':'preempted'},
#                 remapping = {'object_name':'object_to_grasp'})
            
                        # Recognize and pick object if found
                        
                        
                        # Grasp the object
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                pick_object_sm(),
                transitions={'succeeded': 'succeeded', 'aborted': 'say_not_grasp', 
                'preempted': 'preempted'})    
            
            smach.StateMachine.add(
                'recognize_object_and_pick',
                RecObjectAndPick(),
                transitions={'succeeded': 'prepare_delivery_goal', 
                             'fail_grasp':'say_not_grasp',
                             'fail_recognize': 'say_object_not_find'})
    
                        # Go to the delivery place
                        # Grasp Object
            smach.StateMachine.add(
                'say_not_grasp',
                text_to_say("it was impossible to grasp the Object"),
                transitions={'succeeded': 'decide_next_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
                        # Grasp Object
            smach.StateMachine.add(
                'say_object_not_find',
                text_to_say("it was not possible to find the Object"),
                transitions={'succeeded': 'decide_next_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'prepare_delivery_goal',
                prepare_delivery(),
                transitions={'succeeded': 'say_going_deliver_object', 'aborted': 'say_going_deliver_object', 
                'preempted': 'preempted'}) 
            
         # Deliver object
            smach.StateMachine.add(
                'say_going_deliver_object',
                text_to_say("i am going to deliver the object"),
                transitions={'succeeded': 'go_to_delivery', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            # Go to the delivery place
            smach.StateMachine.add(
                'go_to_delivery',
                nav_to_poi(),
                transitions={'succeeded': 'say_deliver_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                        
            # Deliver object
            smach.StateMachine.add(
                'say_deliver_object',
                text_to_say("Delivering Object"),
                transitions={'succeeded': 'deliver_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'deliver_object',
                place_object_sm(pose_place),
                transitions={'succeeded':'check_loop',
                             'aborted':'check_loop',
                             'preempted':'preempted'})
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'decide_next_object', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'succeeded'}) 

