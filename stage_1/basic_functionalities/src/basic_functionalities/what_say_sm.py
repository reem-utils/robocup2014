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

class WhatSaySM(smach.StateMachine):
    """
    Executes a SM that does the test to what did you say.
    The robot listen one question, search in its database and say the answer


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
            
            # Listen the first question
            smach.StateMachine.add(
                'listen_first_question',
                DummyStateMachine(),
                transitions={'succeeded': 'search_first_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Search the answer
            smach.StateMachine.add(
                'search_first_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'say_first_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_first_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'listen_second_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Listen the second question
            smach.StateMachine.add(
                'listen_second_question',
                DummyStateMachine(),
                transitions={'succeeded': 'search_second_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Search the answer
            smach.StateMachine.add(
                'search_second_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'say_second_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_second_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'listen_third_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Listen the third question
            smach.StateMachine.add(
                'listen_third_question',
                DummyStateMachine(),
                transitions={'succeeded': 'search_third_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Search the answer
            smach.StateMachine.add(
                'search_third_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'say_third_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_third_answer',
                DummyStateMachine(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

           

