#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""


import rospy
import smach

from smach_ros import SimpleActionState, ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRSrvResponse, ASREvent, ASRActivation, ASRGrammarMngmt, ASRLanguage
from pal_interaction_msgs.srv import ASRService, ASRServiceRequest, ASRServiceResponse
from util_states.topic_reader import topic_reader_state
from pal_interaction_msgs.msg._ASREvent import ASREvent


"""
    This function will ListenTo a Grammar and get de intresting arg[] for that given grammar
    Right now it just puts on asr_userSaid what the robot has heard without further analysis

sm.userdata.grammar_name = "JungleParty"
smach.StateMachine.add('ListenToTest',
                    ListenToSM(),
                    transitions={'succeeded': 'succe', 'aborted': 'aborted'})

@input string grammar_name
@output string asr_userSaid

"""
# TODO: Add grammar stuff

class Extraction_cb(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=['topic_output_msg'],
                                output_keys=['asr_userSaid','standard_error'])
    
    def execute(self, userdata):
        rospy.loginfo("extracting message from topic")
        userdata.asr_userSaid = userdata.topic_output_msg.recognized_utterance.text
        # TODO: reading tags from grammar recognition
        #userdata.
        userdata.standard_error = ''
    
        return 'succeeded'  
    

class ListenToSM(smach.StateMachine):


    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['grammar_name'],
                    output_keys=['asr_userSaid', 'standard_error'])
        
        with self:
    
  # Call service state
            @smach.cb_interface(input_keys=['grammar_name'])
            def AsrServerRequestActivate_cb(userdata, request):
                rospy.loginfo("Activating Asr server")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.ACTIVATION, ASRSrvRequest.GRAMMAR, ASRSrvRequest.LANGUAGE]
                requ.activation.action = ASRActivation.ACTIVATE                
                requ.grammar.action = ASRGrammarMngmt.ENABLE
                requ.grammar.grammarName = userdata.grammar_name       
                requ.lang.language = 'en_US'
                return requ

            smach.StateMachine.add('Activate_Asr',
                    ServiceState('/asr_server',
                    ASRService,
                    request_cb = AsrServerRequestActivate_cb,
                    input_keys = ['grammar_name']),
                    transitions={'succeeded':'topicReader', 'aborted': 'aborted', 'preempted': 'preempted'})

  # topic reader state
            smach.StateMachine.add('topicReader',
                    topic_reader_state('/asr_event', ASREvent, 2),
                    transitions={'succeeded': 'Process', 'aborted': 'aborted', 'preempted': 'preempted'})

  # Process asr_event -> asr_userSaid state       

            smach.StateMachine.add('Process',
                    Extraction_cb(),
                    transitions={'succeeded': 'Deactivate_Asr'})
            
  # Deactivating asr service to stop listening 
            @smach.cb_interface(input_keys=['grammar_name'])
            def AsrServerRequestDeactivate_cb(userdata, request):
                rospy.loginfo("Dectivating Asr server")
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

