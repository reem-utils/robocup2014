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
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # Prepare the poi for nav_to_poi
            smach.StateMachine.add(
                'prepare_location',
                prepare_location(),
                transitions={'succeeded': 'go_location', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to the location
            smach.StateMachine.add(
                'go_location',
                nav_to_poi(),
                transitions={'succeeded': 'object_recognition', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do object_recognition 
            smach.StateMachine.add(
                'object_recognition',
                DummyStateMachine(),
                transitions={'succeeded': 'grasp_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                DummyStateMachine(),
                transitions={'succeeded': 'go_second_location', 'aborted': 'aborted', 
                'preempted': 'preempted'})     

            # Go the location - We need to go to the place to object category, so we assume that the
            # object recognition will init the poi to the object must to go
            smach.StateMachine.add(
                'go_second_location',
                nav_to_poi(),
                transitions={'succeeded': 'release_object', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Release the object
            smach.StateMachine.add(
                'release_object',
                DummyStateMachine(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

           

