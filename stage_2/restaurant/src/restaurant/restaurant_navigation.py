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
from object_grasping_states.recognize_object import recognize_object

# Constants
NUMBER_OF_ORDERS = 3

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

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
                                output_keys=['standard_error', 'object_name', 'object_loc', 'delivery_loc'])
        

    def execute(self, userdata):
        # We obtain the next element of the array
        
        userdata.object_name = userdata.object_array[userdata.object_index][0]
        userdata.object_loc = userdata.object_array[userdata.object_index][1]
        userdata.delivery_loc = userdata.object_array[userdata.object_index][2]
        
        rospy.loginfo("Object_name: " + userdata.object_name)
        rospy.loginfo("Object_loc: " + userdata.object_loc)
        rospy.loginfo("delivery_loc " + userdata.delivery_loc)
        
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

    Given a array with the information {objectName, objectLocation, deliveryLocation}
    the robot goes to the objectLocation, take it and deliver to the location. 
    It does the same while the array has objects.  
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['object_array'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.object_name = ''
            self.userdata.object_index = 0

            # Process order
            smach.StateMachine.add(
                'decide_next_object',
                decide_next_object(),
                transitions={'succeeded': 'go_to_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Go to poi where the object can stay
            smach.StateMachine.add(
                'go_to_object',
                nav_to_poi(),
                remapping={"nav_to_poi_name": "object_loc"},
                transitions={'succeeded': 'object_detection', 'aborted': 'aborted'})
                        
            # Object Detection
            smach.StateMachine.add(
                'object_detection',
                recognize_object(),
                transitions={'succeeded': 'grasp_object', 'aborted': 'aborted'})

            # Grasp Object
            smach.StateMachine.add(
                'grasp_object',
                DummyStateMachine("Grasping Object"),
                transitions={'succeeded': 'go_to_delivery', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Go to the delivery place
            smach.StateMachine.add(
                'go_to_delivery',
                nav_to_poi(),
                remapping = {'nav_to_poi_name': 'delivery_loc'},
                transitions={'succeeded': 'deliver_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                        
            # Deliver object
            smach.StateMachine.add(
                'deliver_object',
                DummyStateMachine("Delivering Object"),
                transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'decide_next_object', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'succeeded'}) 

