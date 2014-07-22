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
from activate_asr import ActivateASR
from deactivate_asr import DeactivateASR
from read_asr import ReadASR
from asr_calibrate import CalibrateASR

class saveData(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['asr_userSaid', 'asr_userSaid_tags'], output_keys=['asr_answer', 'asr_answer_tags', 'tts_text'])

        
    def execute(self, userdata):

        userdata.asr_answer = userdata.asr_userSaid 
        userdata.asr_answer_tags = userdata.asr_userSaid_tags  
        userdata.tts_text = "Did you said " + userdata.asr_userSaid + "?"
 
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
    
 
class check(smach.State):
    
    def __init__(self, calibrate):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=[], output_keys=[])
        self.calibrate=calibrate
    def execute(self, userdata):
           
        if self.calibrate==True :
            return 'succeeded'
        else :
            return 'aborted'

    
    
class AskQuestionSM(smach.StateMachine):

    """      
        This function will do a question, listen the answer and then confirm it.
        If return succeeded, the confirm answer was yes. Otherwise, it return aborted
        
        @input string text or tts_text
        @output string asr_answer
        @output actiontag[] asr_answer_tags
    """
    
    def __init__(self, text=None, grammar = None, calibrate=False,Bucle=True,time_calibrate=12):
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
                 transitions={'succeeded':'ActivateASR', 'aborted':'aborted'})
            
            # Activate the asr
            smach.StateMachine.add('ActivateASR',
                 ActivateASR(),
                 transitions={'succeeded': 'ask_question', 'aborted': 'aborted', 'preempted': 'preempted'})                    
            
            # Ask for order
            smach.StateMachine.add(
                'ask_question',
                text_to_say(),
                transitions={'succeeded': 'listen_answer', 'aborted': 'aborted', 'preempted': 'preempted'})

            # Listen the order and repeat
            smach.StateMachine.add(
                'listen_answer',
                ReadASR(Time=12,bucle=Bucle,Time_calibrate=time_calibrate),
                transitions={'succeeded': 'SaveData', 'aborted': 'calibrate', 'preempted': 'preempted'})
           
            # Save information
            smach.StateMachine.add(
                'SaveData',
                saveData(),
                transitions={'succeeded':'ActivateASR_yesno', 'aborted':'aborted'})
            
            # Load grammar yes/no
            smach.StateMachine.add(
                'ActivateASR_yesno',
                ActivateASR("robocup/yes_no"),
                transitions={'succeeded': 'repeat_info', 'aborted': 'aborted', 'preempted': 'preempted'})                    
            
            # Repeat the question
            smach.StateMachine.add(
                'repeat_info',
                text_to_say(),
                transitions={'succeeded': 'confirm_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Confirm order
            smach.StateMachine.add(
                'confirm_question',
                SayYesOrNoSM(bucle=bucle,calibrate=calibrate,Time_calibrate=Time_calibrate),
                transitions={'succeeded': 'DeactivateASR', 'aborted': 'calibrate', 'preempted': 'preempted'})

            # Deactivate the server
            smach.StateMachine.add('DeactivateASR',
                DeactivateASR(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            
            
            smach.StateMachine.add(
                'calibrate',
                check(calibrate),
                transitions={'succeeded': 'calibrate_action', 'aborted': 'aborted', 
                'preempted': 'aborted'})
            
            
            # all the places that was aborted now is calibrate
            smach.StateMachine.add(
                'calibrate_action',
                CalibrateASR(),
                transitions={'succeeded': 'aborted', 'aborted': 'aborted', 
                'preempted': 'aborted'})
