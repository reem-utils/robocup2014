#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 06/04/2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach

from speech_states.listen_to import ListenToSM

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
           
class prepareData(smach.State):
    
    def __init__(self, word, grammar):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['word_to_listen','grammar_name'], output_keys=['word_to_listen', 'grammar_name'])
        self.word = word
        self.grammar = grammar
        
    def execute(self, userdata):
        
        if not self.word and not userdata.word_to_listen:
            rospy.logerr("Word isn't set")
            return 'aborted'
                
        if not self.grammar and not userdata.grammar_name:
            rospy.logerr("Grammar isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.word_to_listen = self.word if self.word else userdata.word_to_listen
        userdata.grammar_name = self.grammar if self.grammar else userdata.grammar_name
 
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
        @input string word or word_to_listen

    """
    
    def __init__(self, word=None, gram=''):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['word_to_listen', 'grammar_name'],
                    output_keys=[])
        
        with self:
            self.userdata.listen_word = None
            self.userdata.grammar_name = None
    
            smach.StateMachine.add('PrepareData',
                    prepareData(word, gram),
                    transitions={'succeeded':'listen_word', 'aborted':'aborted'})
             
            # Listen the word
            smach.StateMachine.add(
                    'listen_word',
                    ListenToSM(),
                    transitions={'succeeded': 'checkData', 'aborted': 'aborted', 'preempted': 'preempted'})

            # Check information
            smach.StateMachine.add('checkData',
                    checkData(),
                    transitions={'succeeded':'succeeded', 'aborted':'listen_word'})