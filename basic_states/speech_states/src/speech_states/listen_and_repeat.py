#! /usr/bin/env python
'''
Created on 28/03/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
import os
import sys
from speech_states.say import text_to_say
from speech_states.listen_to import ListenToSM

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
    
class prepare_say(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['asr_userSaid'],
                                output_keys=['tts_text','tts_wait_before_speaking'])

    def execute(self, userdata):

        userdata.tts_text = "Did you say " + userdata.asr_userSaid + "?"
        userdata.tts_wait_before_speaking = 0

        return 'succeeded'
 
class prepareData(smach.State):
    
    def __init__(self, grammar):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['grammar_name'], output_keys=['grammar_name'])
        self.grammar = grammar
        
    def execute(self, userdata):
           
        if not self.grammar and not userdata.grammar_name:
            rospy.logerr("Grammar_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.grammar_name = self.grammar if self.grammar else userdata.grammar_name   
 
        return 'succeeded'
       
class ListenRepeatSM(smach.StateMachine):
    """
    Executes a SM that learns one face. 
    The robot listen the person and later repeat. It needs a grammar. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self, grammar = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                        input_keys=['grammar_name'],
                                        output_keys=['asr_userSaid'])

        with self:
            self.userdata.asr_userSaid = ''
            self.userdata.tts_lang = None
            
            smach.StateMachine.add('PrepareData',
                    prepareData(grammar),
                    transitions={'succeeded':'listen_info', 'aborted':'aborted'})
             
            # Listen             
            smach.StateMachine.add(
                'listen_info',
                ListenToSM(),
                transitions={'succeeded': 'prepare_info', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Prepare the information 
            smach.StateMachine.add(
                "prepare_info",
                prepare_say(),
                transitions={'succeeded': 'say_info', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
           
            # Repeat
            smach.StateMachine.add(
                'say_info',
                text_to_say(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
