#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author: Cristina De Saint Germain
"""

import rospy
import smach
import copy

from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRActivation, ASRLangModelMngmt
from pal_interaction_msgs.srv import ASRService
from util_states.sleeper import Sleeper

class prepareData(smach.State):
    
    def __init__(self, keyword):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['keyword_name'], output_keys=['keyword_name'])
        self.keyword = keyword
        
    def execute(self, userdata):
           
        if not self.keyword and not userdata.keyword_name:
            rospy.logerr("keyword_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.keyword_name = self.keyword if self.keyword else userdata.keyword_name   
 
        return 'succeeded'
    
class ActivateKeywordASR(smach.StateMachine):

    """
        This state machine activate the ASR service. It needs a grammar.
    
        smach.StateMachine.add('ActivateASRTest',
                        ActivateASR("JungleParty"),
                        transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    
        @input string grammar_name
    
    """
    def __init__(self, keyword = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['keyword_name'],
                    output_keys=[])
        
        with self:
    
            smach.StateMachine.add('PrepareData',
                                   prepareData(keyword),
                                   transitions={'succeeded':'Activate_keyword', 'aborted':'aborted'})
            # Call service state
            @smach.cb_interface(input_keys=['keyword_name'])
            def AsrServerActivateKeyword_cb(userdata, request):
                rospy.loginfo("Activating Keyword " + userdata.keyword_name)
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.KWSPOTTING]                
                requ.model.action = ASRLangModelMngmt.ENABLE
                requ.model.modelName = copy.deepcopy(userdata.keyword_name)
                rospy.logwarn("AsrServerActivateKeyword_cb sending keyword: \n" + str(requ.model.modelName))
                return requ

            def AsrServerActivateKeyword_response_cb(userdata, response):
                rospy.logwarn("response of Activate_keyword is: \n" + str(response))
                return 'succeeded'

            smach.StateMachine.add('Activate_keyword',
                    ServiceState('/asr_service',
                    ASRService,
                    request_cb = AsrServerActivateKeyword_cb,
                    response_cb = AsrServerActivateKeyword_response_cb,
                    input_keys = ['keyword_name']),
                    transitions={'succeeded':'Sleeper', 'aborted': 'aborted', 'preempted': 'preempted'})
             
            smach.StateMachine.add("Sleeper", Sleeper(0.1), 
                                   transitions={'succeeded':'Activate_Asr', 'aborted': 'aborted'})
            # Call service state
            @smach.cb_interface(input_keys=['keyword_name'])
            def AsrServerRequestActivate_cb(userdata, request):
                rospy.loginfo("Activating Asr service")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.KWSPOTTING, ASRSrvRequest.ACTIVATION]
                requ.activation.action = ASRActivation.ACTIVATE 
                requ.model.action = ASRLangModelMngmt.ENABLE
                requ.model.modelName = copy.deepcopy(userdata.keyword_name)
                rospy.logwarn("AsrServerRequestActivate_cb activating keyword: " + str(requ.model.modelName))
                return requ

            def AsrServerRequestActivate_response_cb(userdata, response):
                rospy.logwarn("response of Activate_Asr is: \n" + str(response))
                return 'succeeded'

            smach.StateMachine.add('Activate_Asr',
                    ServiceState('/asr_service',
                    ASRService,
                    request_cb = AsrServerRequestActivate_cb,
                    response_cb = AsrServerRequestActivate_response_cb,
                    input_keys = ['keyword_name']),
                    transitions={'succeeded':'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
             



