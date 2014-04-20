#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach

from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRActivation, ASRLangModelMngmt
from pal_interaction_msgs.srv import ASRService
from util_states.sleeper import Sleeper

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
    
class ActivateASR(smach.StateMachine):

    """
        This state machine activate the ASR service. It needs a grammar.
    
        smach.StateMachine.add('ActivateASRTest',
                        ActivateASR("JungleParty"),
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
                                   transitions={'succeeded':'Activate_Gram', 'aborted':'aborted'})
            # Call service state
            @smach.cb_interface(input_keys=['grammar_name'])
            def AsrServerActivateGram_cb(userdata, request):
                rospy.loginfo("Activating Grammar " + userdata.grammar_name + " and Language")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.GRAMMAR, ASRSrvRequest.LANGUAGE]                
                requ.model.action = ASRLangModelMngmt.ENABLE
                requ.model.modelName = userdata.grammar_name       
                requ.lang.language = 'en_US'
                return requ

            smach.StateMachine.add('Activate_Gram',
                    ServiceState('/asr_service',
                    ASRService,
                    request_cb = AsrServerActivateGram_cb,
                    input_keys = ['grammar_name']),
                    transitions={'succeeded':'Sleeper', 'aborted': 'aborted', 'preempted': 'preempted'})
             
            smach.StateMachine.add("Sleeper", Sleeper(3), 
                                   transitions={'succeeded':'Activate_Asr', 'aborted': 'aborted'})
            # Call service state
            def AsrServerRequestActivate_cb(userdata, request):
                rospy.loginfo("Activating Asr service")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.ACTIVATION]
                requ.activation.action = ASRActivation.ACTIVATE     
                return requ

            smach.StateMachine.add('Activate_Asr',
                    ServiceState('/asr_service',
                    ASRService,
                    request_cb = AsrServerRequestActivate_cb),
                    transitions={'succeeded':'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
             



