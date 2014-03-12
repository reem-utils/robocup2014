#! /usr/bin/env python

import rospy
import smach


from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRSrvResponse, ASREvent, ASRActivation, ASRGrammarMngmt, ASRLanguage, actiontag
from pal_interaction_msgs.srv import ASRService, ASRServiceRequest, ASRServiceResponse



class AsrStatus(smach.StateMachine):
    """
    ASR Status
    
    @output string asr_status
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 output_keys=['standard_error', 'asr_status'])
        
        with self:
            
            def AsrServerRequestStatus_cb(userdata, request):
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.STATUS]
                requ.activation.action = ASRActivation.ACTIVATE                
                requ.grammar.action = ASRGrammarMngmt.ENABLE
                requ.grammar.grammarName = ''       
                requ.lang.language = 'en_US'
                userdata.standard_error = 'ok'
                return requ
            
            def asr_status_cb(userdata, resp): 
                rospy.loginfo(resp.response.status.active)                
                rospy.loginfo(resp.response.status.enabled_grammar)
                rospy.loginfo(resp.response.status.language)
                rospy.loginfo(resp.response.error_msg)
                rospy.loginfo(resp.response.warn_msg)
                self.userdata.asr_status = resp
#                 

            smach.StateMachine.add('statusSM',
                    ServiceState('/asr_server',
                    ASRService,
                    request_cb = AsrServerRequestStatus_cb,
                    response_cb = asr_status_cb,
                    output_keys = ['standard_error']),
                    transitions={'succeeded':'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
            
#                  
#             smach.StateMachine.add('statusSM',
#                                   ServiceState('/asr_server',
#                                                ASRService,
#                                                response_cb = asr_status_cb,
#                                                output_keys = ['standard_error']),
#                                   transitions={'succeeded':'succeeded','aborted': 'aborted','preempted':'preempted'})
#        


