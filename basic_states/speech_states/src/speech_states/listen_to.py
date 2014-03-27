#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach

from activate_asr import ActivateASR
from deactivate_asr import DeactivateASR
from read_asr import ReadASR

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
    
class ListenToSM(smach.StateMachine):

    """      
        This function will ListenTo a Grammar and get the interesting arg[] for that given grammar
        Right now it just puts on asr_userSaid what the robot has heard without further analysis
    
        smach.StateMachine.add('ListenToTest',
                        ListenToSM("JungleParty"),
                        transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    
        @input string grammar_name
        @output string asr_userSaid
        @output actiontag[] asr_userSaid_tags
    """
    
    def __init__(self, grammar=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['grammar_name'],
                    output_keys=['asr_userSaid', 'standard_error', 'asr_userSaid_tags'])
        
        with self:
            
            smach.StateMachine.add('PrepareData',
                    prepareData(grammar),
                    transitions={'succeeded':'ActivateASR', 'aborted':'aborted'})
             
            # Activate the server
            smach.StateMachine.add('ActivateASR',
                    ActivateASR(),
                    transitions={'succeeded': 'ReadASR', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Read from server
            smach.StateMachine.add('ReadASR',
                    ReadASR(),
                    transitions={'succeeded': 'DeactivateASR', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Deactivate the server
            smach.StateMachine.add('DeactivateASR',
                    DeactivateASR(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
            

