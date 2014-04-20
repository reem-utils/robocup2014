#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
2 April 2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach

from speech_states.say import text_to_say
from speech_states.listen_and_repeat import ListenRepeatSM
from speech_states.say_yes_or_no import SayYesOrNoSM

class saveData(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['asr_userSaid', 'asr_userSaid_tags'], output_keys=['asr_answer', 'asr_answer_tags'])

        
    def execute(self, userdata):

        userdata.asr_answer = userdata.asr_userSaid 
        userdata.asr_answer_tags = userdata.asr_userSaid_tags  
 
        return 'succeeded'
    
class prepareData(smach.State):
    
    def __init__(self, text, grammar):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['tts_text', 'grammar_name'], output_keys=['tts_text', 'grammar_name'])
        self.text = text
        self.grammar = grammar
        
    def execute(self, userdata):
           
        if not self.text and not userdata.tts_text:
            rospy.logerr("Text isn't set")
            return 'aborted'
        
        if not self.grammar and not userdata.grammar_name:
            rospy.logerr("Grammar_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.grammar_name = self.grammar if self.grammar else userdata.grammar_name 
        userdata.tts_text = self.text if self.text else userdata.tts_text  
 
        return 'succeeded'
    
class AskQuestionSM(smach.StateMachine):

    """      
        This function will do a question, listen the answer and then confirm it.
        If return succeeded, the confirm answer was yes. Otherwise, it return aborted
        
        @input string text or tts_text
        @output string asr_answer
        @output actiontag[] asr_answer_tags
    """
    
    def __init__(self, text=None, grammar = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['tts_text', 'grammar_name'],
                    output_keys=['asr_answer', 'standard_error', 'asr_answer_tags'])
        
        with self:
            self.userdata.tts_text = None
            self.userdata.tts_wait_before_speaking = None
            self.userdata.tts_lang = None
            self.userdata.grammar_name = None
    
            smach.StateMachine.add('PrepareData',
                    prepareData(text, grammar),
                    transitions={'succeeded':'ask_question', 'aborted':'aborted'})
             
            # Ask for order
            smach.StateMachine.add(
                    'ask_question',
                    text_to_say(),
                    transitions={'succeeded': 'listen_repeat_question', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Listen the order and repeat
            smach.StateMachine.add(
                    'listen_repeat_question',
                    ListenRepeatSM(),
                    transitions={'succeeded': 'SaveData', 'aborted': 'aborted', 'preempted': 'preempted'})
           
            # Save information
            smach.StateMachine.add('SaveData',
                    saveData(),
                    transitions={'succeeded':'confirm_question', 'aborted':'aborted'})
            
            # Confirm order
            smach.StateMachine.add(
                    'confirm_question',
                    SayYesOrNoSM(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})

