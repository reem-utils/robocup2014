#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

@author: Sergi Xavier Ubach Pallàs
@email: sxubach@gmail.com

26 Feb 2014
"""

import rospy
import smach
from speech_states.say  import text_to_say
from speech_states.listen_to import ListenToSM

# Constants
NUMBER_OF_QUESTIONS = 3
GRAMMAR_NAME = 'what_did_you_say.gram'

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
    
class SelectAnswer(smach.State):
    def __init__(self):
        rospy.loginfo("Entring SelectAnswer")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'None'], 
                                input_keys=['asr_userSaid', 'asr_userSaid_tags'],
                                output_keys=['standard_error', 'tts_text', 'tts_wait_before_speaking'])

    def execute(self, userdata):        
        question = userdata.asr_userSaid
        questionTags = userdata.asr_userSaid_tags
        foundAnswer = False
        #important to do add the .yalm before
        question_params = rospy.get_param("/question_list/questions/what_say")
       
        info = [tag for tag in questionTags if tag.key == 'info']
        country = [tag for tag in questionTags if tag.key == 'country']
        
        for key, value in question_params.iteritems():
            
            if (info and info[0].value == value[2]) and (country and country[0].value == value[3]):
                userdata.tts_text = value[4]
                userdata.tts_wait_before_speaking = 0
                foundAnswer = True
                break

        if foundAnswer:
            userdata.standard_error='OK'
#             userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'
#         elif userdata.loop_iterations == 10:
#             return 'preempted'        
        else:
            
            userdata.tts_text = "I don't know"
            userdata.tts_wait_before_speaking = 0
            userdata.standard_error='Answer not found'
            rospy.loginfo( FAIL +'ANSWER NOT FOUND') # todo change to loginfo
            return 'succeeded'
        
        

class loopTest(smach.State):
    def __init__(self):
        rospy.loginfo("Entring loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['loop_iterations'],
                                output_keys=['standard_error', 'loop_iterations', 'tts_text', 'tts_wait_before_speaking'])

    def execute(self, userdata):
        
        userdata.tts_text = "I'm ready, ask me a question"
        userdata.tts_wait_before_speaking = 0
        
        if userdata.loop_iterations == NUMBER_OF_QUESTIONS:
            return 'aborted'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.loop_iterations = userdata.loop_iterations + 1
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

    To use this SM in a simulator is required to run asr_srv.py, tts_as and roscore.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 0
            # Listen the first question
            self.userdata.grammar_name = GRAMMAR_NAME
            
            # loop test
            smach.StateMachine.add(
                'loop_test',
                loopTest(),
                transitions={'succeeded': 'ask_next_question', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # Ask for next question
            smach.StateMachine.add(
                'ask_next_question',
                text_to_say(),
                transitions={'succeeded': 'listen_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'listen_question',
                ListenToSM(),
                transitions={'succeeded': 'search_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            # TODO: stop listenting after getting the sentence. toask

            # Search the answer
            smach.StateMachine.add(
                'search_answer',
                SelectAnswer(),
                transitions={'succeeded': 'say_answer', 'aborted': 'aborted', 
                'preempted': 'preempted', 'None': 'aborted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_answer',
                text_to_say(),
                transitions={'succeeded': 'loop_test', 'aborted': 'aborted', 
                'preempted': 'preempted'})  


            
            