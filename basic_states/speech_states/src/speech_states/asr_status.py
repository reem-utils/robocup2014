#! /usr/bin/env python

"""
Author:  Sergi Xavier Ubach
Email: sxubach@gmail.com

22 Feb 2014
"""

import rospy
import smach

from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRSrvResponse, ASREvent, ASRActivation, ASRGrammarMngmt, ASRLanguage, actiontag
from pal_interaction_msgs.srv import ASRService, ASRServiceRequest, ASRServiceResponse

class AsrStatus(smach.StateMachine):
    """
    ASR Status. 

    Return the Status from ASR Server. The params that shows are:
	Active, Enabled_grammar, Language, Err_msg, War_msg.
    It shows the status on screen too.
    
    @output string asr_status: Contains the status from ASRServer
    @output string standard_error: Shows the info from err_msg
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 output_keys=['standard_error', 'asr_status'])
        
        with self:
            
	        # Request
            def AsrServerRequestStatus_cb(userdata, request):
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.STATUS]
                requ.activation.action = ASRActivation.ACTIVATE                
                requ.grammar.action = ASRGrammarMngmt.ENABLE
                requ.grammar.grammarName = ''       
                requ.lang.language = 'en_US'
                userdata.standard_error = ''
                return requ
            
	        # Callback
            def asr_status_cb(userdata, resp): 
                rospy.loginfo(resp.response.status.active)                
                rospy.loginfo(resp.response.status.enabled_grammar)
                rospy.loginfo(resp.response.status.language)
                rospy.loginfo(resp.response.error_msg)
                rospy.loginfo(resp.response.warn_msg)
                self.userdata.standard_error = resp.response.error_msg
                self.userdata.asr_status = resp              

            smach.StateMachine.add('ASRStatus',
                    ServiceState('/asr_server',
                    ASRService,
                    request_cb = AsrServerRequestStatus_cb,
                    response_cb = asr_status_cb,
                    output_keys = ['standard_error']),
                    transitions={'succeeded':'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
            


