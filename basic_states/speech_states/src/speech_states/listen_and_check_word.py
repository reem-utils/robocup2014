#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 06/04/2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach

from speech_states.listen_to import ListenToSM

from activate_keyword_asr import ActivateKeywordASR
from deactivate_asr import DeactivateASR
from read_asr import ReadASR

class checkData(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['asr_userSaid', 'asr_userSaid_tags', 'word_to_listen'],
                             output_keys=[])

        
    def execute(self, userdata):

        if self.preempt_requested():
            return 'preempted'
       
        word = [tag for tag in userdata.asr_userSaid_tags if tag.key == 'action']
        
        if word and word[0].value == userdata.word_to_listen:
            rospy.loginfo("Match!")
            return 'succeeded'
        else:
            rospy.sleep(0.2)
            return 'aborted'
class checkData_concurrent(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['asr_userSaid', 'asr_userSaid_tags', 'word_to_listen'],
                             output_keys=[])

        
    def execute(self, userdata):
            #TODO: we don't comprovate the word that we listen because it's only possible one word
            return 'succeeded'
            rospy.sleep(0.2)
            return 'aborted'
           
class prepareData(smach.State):
    
    def __init__(self, word):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['word_to_listen','grammar_name'], output_keys=['word_to_listen', 'grammar_name'])
        self.word = word
        
    def execute(self, userdata):
        
        if not self.word and not userdata.word_to_listen:
            rospy.logerr("Word isn't set")
            return 'aborted'
       
        #Priority in init
        userdata.word_to_listen = self.word if self.word else userdata.word_to_listen
 
        return 'succeeded'


class prepareDataKeyword(smach.State):
    
    def __init__(self, word):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['keyword_name'], output_keys=['keyword_name'])
        self.word = word
        
    def execute(self, userdata):
        
        if not self.word and not userdata.keyword_name:
            rospy.logerr("Word isn't set")
            return 'aborted'
                
        #Priority in init
        userdata.keyword_name = self.word if self.word else userdata.keyword_name
 
        return 'succeeded'
        
class ListenWordSM(smach.StateMachine):

    """      
        This StateMachine listen a word and compare if it is the desired word.
        It returns succeeded if the word matches. Otherwise, aborted. 
        
        @input string word or word_to_listen

    """
    
    def __init__(self, word=None, gram_name=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['word_to_listen', 'grammar_name'],
                    output_keys=[])
        
        with self:
            self.userdata.listen_word = None
            self.userdata.grammar_name = None
    
            smach.StateMachine.add('PrepareData',
                    prepareData(word, gram_name),
                    transitions={'succeeded':'listen_word', 'aborted':'aborted'})
             
            # Listen the word
            smach.StateMachine.add(
                    'listen_word',
                    ListenToSM(),
                    transitions={'succeeded': 'checkData', 'aborted': 'aborted', 'preempted': 'preempted'})
           
            # Check information
            smach.StateMachine.add('checkData',
                    checkData(),
                    transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            
            
            
class ListenWordSM_Concurrent(smach.StateMachine):

    """      
        This StateMachine listen a word and compare if it is the desired word.
        It returns succeeded if the word matches. Otherwise it will be waiting for
        the correct word
        @input string word or keyword_name

    """
    
    def __init__(self, word=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['keyword_name'],
                    output_keys=[])
        
        with self:
            self.userdata.keyword_name = word
            self.userdata.word_to_listen = None
    
            smach.StateMachine.add('PrepareData',
                    prepareData(word),
                    transitions={'succeeded':'DeactivateASR_start', 'aborted':'aborted'})
             
            # Deactivate the server
            smach.StateMachine.add('DeactivateASR_start',
                    DeactivateASR(),
                    transitions={'succeeded': 'ActivateASR', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Activate the server
            smach.StateMachine.add('ActivateASR',
                    ActivateKeywordASR(),
                    transitions={'succeeded': 'ReadASR', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Read from server
            smach.StateMachine.add('ReadASR',
                    ReadASR(),
                    transitions={'succeeded': 'DeactivateASR', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Deactivate the server
            smach.StateMachine.add('DeactivateASR',
                    DeactivateASR(),
#                     transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
#               
#             # Listen the word
#             smach.StateMachine.add(
#                     'listen_word',
#                     ListenToSM(),
                    transitions={'succeeded': 'checkData', 'aborted': 'aborted', 'preempted': 'preempted'})

            # Check information
            smach.StateMachine.add('checkData',
                    checkData_concurrent(),
                    transitions={'succeeded':'succeeded', 'aborted':'aborted'})