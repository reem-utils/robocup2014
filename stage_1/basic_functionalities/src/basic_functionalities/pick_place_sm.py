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
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'

# Class that prepare the value need for nav_to_poi
class prepare_location(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='pick_and_place'
        return 'succeeded'

class PickPlaceSM(smach.StateMachine):
    """
    Executes a SM that does the test to pick and place.
    The robot goes to a location and recognize one object.
    It picks the object and goes a location where it will be release. 


    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """say_going_pick_place
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # Say start Pick and Place
            smach.StateMachine.add(
                 'say_start_pick_place',
                 text_to_say("I'm going to start Pick and Place test"),
                 transitions={'succeeded': 'say_go_location', 'aborted': 'say_go_location'}) 
            
            # Say Going to location
            smach.StateMachine.add(
                 'say_go_location',
                 text_to_say("I'm going to the location where I have to recognize some objects"),
                 transitions={'succeeded': 'prepare_location', 'aborted': 'prepare_location'}) 
            
            # Prepare the poi for nav_to_poi
            smach.StateMachine.add(
                'prepare_location',
                prepare_location(),
                transitions={'succeeded': 'go_location', 'aborted': 'prepare_location', 
                'preempted': 'preempted'})  

            # Go to the location
            smach.StateMachine.add(
                'go_location',
                nav_to_poi(),
                transitions={'succeeded': 'say_start_obj_recognition', 'aborted': 'say_go_location', 
                'preempted': 'preempted'})    

            # Say start object recognition
            smach.StateMachine.add(
                 'say_start_obj_recognition',
                 text_to_say("I'm going to start the Object recognition"),
                 transitions={'succeeded': 'object_recognition', 'aborted': 'object_recognition'}) 
            
            # Do object_recognition 
            smach.StateMachine.add(
                'object_recognition',
                DummyStateMachine(),
                transitions={'succeeded': 'say_grasp_object', 'aborted': 'say_grasp_object', 
                'preempted': 'preempted'}) 

            # Say grasp object
            smach.StateMachine.add(
                 'say_grasp_object',
                 text_to_say("I'm going to grasp the object"),
                 transitions={'succeeded': 'grasp_object', 'aborted': 'grasp_object'})
            
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                DummyStateMachine(),
                transitions={'succeeded': 'say_go_second_location', 'aborted': 'say_go_second_location', 
                'preempted': 'preempted'})     

            # Say go to second location
            smach.StateMachine.add(
                 'say_go_second_location',
                 text_to_say("I'm going to the location where I should release the object"),
                 transitions={'succeeded': 'go_second_location', 'aborted': 'go_second_location'})
            
            # Go the location - We need to go to the place to object category, so we assume that the
            # object recognition will init the poi to the object must to go
            smach.StateMachine.add(
                'go_second_location',
                nav_to_poi(),
                transitions={'succeeded': 'say_release_obj', 'aborted': 'say_go_second_location', 
                'preempted': 'preempted'}) 

            # Say release object
            smach.StateMachine.add(
                 'say_release_obj',
                 text_to_say("I'm going to release the object"),
                 transitions={'succeeded': 'release_object', 'aborted': 'release_object'})
            
            # Release the object
            smach.StateMachine.add(
                'release_object',
                DummyStateMachine(),
                transitions={'succeeded': 'say_end_pick_place', 'aborted': 'say_end_pick_place', 
                'preempted': 'preempted'})    

            # Say end Pick and Place
            smach.StateMachine.add(
                 'say_end_pick_place',
                 text_to_say("I finished the Pick and Place test"),
                 transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
           

