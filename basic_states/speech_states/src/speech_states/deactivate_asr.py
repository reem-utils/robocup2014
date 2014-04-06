#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach

from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRActivation, ASRGrammarMngmt
from pal_interaction_msgs.srv import ASRService

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
    

class DeactivateASR(smach.StateMachine):

    """
    This state machine deactivate the ASR Server. It needs a grammar.

    smach.StateMachine.add('ActivateASRTest',
                    DeactivateASR("JungleParty"),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    @input string grammar_name

    """
    def __init__(self, grammar = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['grammar_name'],
                    output_keys=[])
        
        with self:
    
            smach.StateMachine.add('PrepareData',
                                   prepareData(grammar),
                                   transitions={'succeeded':'Deactivate_Asr', 'aborted':'aborted'})
            
            # Deactivating asr service to stop listening 
            @smach.cb_interface(input_keys=['grammar_name'])
            def AsrServerRequestDeactivate_cb(userdata, request):
                rospy.loginfo("Deactivating Asr server")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.ACTIVATION, ASRSrvRequest.GRAMMAR, ASRSrvRequest.LANGUAGE]
                requ.activation.action = ASRActivation.DEACTIVATE                
                requ.grammar.action = ASRGrammarMngmt.ENABLE
                requ.grammar.grammarName = userdata.grammar_name       
                requ.lang.language = 'en_US'
                return requ
            
            smach.StateMachine.add('Deactivate_Asr',
                    ServiceState('/asr_server',
                    ASRService,
                    request_cb = AsrServerRequestDeactivate_cb,
                    input_keys = ['grammar_name']),
                    transitions={'succeeded':'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})


