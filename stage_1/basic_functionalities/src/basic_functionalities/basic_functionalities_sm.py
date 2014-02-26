#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com
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
class prepare_pick_and_place(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='init_pick_and_place'
        return 'succeeded'

class prepare_fetch_and_carry(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='init_fetch_and_carry'
        return 'succeeded'

class prepare_find_me(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='init_find_me'
        return 'succeeded'

class prepare_avoid_that(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='init_avoid_that'
        return 'succeeded'

class prepare_what_did_you_say(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='init_what_say'
        return 'succeeded'

class BasicFuncionalitiesSM(smach.StateMachine):
    """
    Executes a SM that does the basic funcionalities.
    The robot moves autonomously to each activities 


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
            
            # Prepare the poi for pick and place
            smach.StateMachine.add(
                'prepare_pick_and_place',
                prepare_pick_and_place(),
                transitions={'succeeded': 'go_pick_and_place', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to pick and place
            smach.StateMachine.add(
                'go_pick_and_place',
                nav_to_poi(),
                transitions={'succeeded': 'do_pick_and_place', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do pick and place
            smach.StateMachine.add(
                'do_pick_and_place',
                DummyStateMachine(),
                transitions={'succeeded': 'prepare_fetch_and_carry', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Prepare the poi for Fetch and Carry
            smach.StateMachine.add(
                'prepare_fetch_and_carry',
                prepare_fetch_and_carry(),
                transitions={'succeeded': 'go_fetch_and_carry', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to fetch and carry
            smach.StateMachine.add(
                'go_fetch_and_carry',
                nav_to_poi(),
                transitions={'succeeded': 'do_fetch_and_carry', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do fetch and carry
            smach.StateMachine.add(
                'do_fetch_and_carry',
                DummyStateMachine(),
                transitions={'succeeded': 'prepare_find_me', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Prepare the poi for find me
            smach.StateMachine.add(
                'prepare_find_me',
                prepare_find_me(),
                transitions={'succeeded': 'go_find_me', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to find me
            smach.StateMachine.add(
                'go_find_me',
                nav_to_poi(),
                transitions={'succeeded': 'do_find_me', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do find me
            smach.StateMachine.add(
                'do_find_me',
                DummyStateMachine(),
                transitions={'succeeded': 'prepare_avoid_that', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Prepare the poi for avoid that
            smach.StateMachine.add(
                'prepare_avoid_that',
                prepare_avoid_that(),
                transitions={'succeeded': 'go_avoid_that', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to avoid that
            smach.StateMachine.add(
                'go_avoid_that',
                nav_to_poi(),
                transitions={'succeeded': 'do_avoid_that', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do avoid that
            smach.StateMachine.add(
                'do_avoid_that',
                DummyStateMachine(),
                transitions={'succeeded': 'prepare_what_did_you_say', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Prepare the poi for what did you say
            smach.StateMachine.add(
                'prepare_what_did_you_say',
                prepare_what_did_you_say(),
                transitions={'succeeded': 'go_door_in', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to what did you say
            smach.StateMachine.add(
                'go_what_did_you_say',
                nav_to_poi(),
                transitions={'succeeded': 'do_what_did_you_say', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do what did you say
            smach.StateMachine.add(
                'do_what_did_you_say',
                DummyStateMachine(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 


