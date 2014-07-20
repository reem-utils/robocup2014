#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue July 9 9:54:00 PM 2014

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""

import rospy
import smach
import copy

from smach_ros import ServiceState
from pal_interaction_msgs.msg import ASRSrvRequest, ASRActivation
from pal_interaction_msgs.srv import ASRService
from util_states.sleeper import Sleeper
from speech_states.say import text_to_say
SAY_CALIBRING= "I have to recalibrate the microfon, it's to much noise here"

class waitstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
    def execute(self, userdata):
        rospy.sleep(1)
        return 'succeeded'
    
class CalibrateASR(smach.StateMachine):

    """
        This state machine calibrates the ASR service.
    
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=[],
                    output_keys=[])
        
        with self:
   
            # Call service state
            def AsrServerRequestActivate_cb(userdata, request):
                rospy.loginfo("Activating Asr service")
                requ = ASRSrvRequest()
                requ.requests = [ASRSrvRequest.ACTIVATION]
                requ.activation.action = ASRActivation.CALIBRATE 
                #requ.model.action = ASRLangModelMngmt.ENABLE
                #requ.model.modelName = copy.deepcopy(userdata.grammar_name)
                rospy.logwarn("AsrServerRequestActivate_cb Calibrating")
                return requ

            def AsrServerRequestActivate_response_cb(userdata, response):
                rospy.logwarn("response of Activate_Asr is: \n" + str(response))
                return 'succeeded'
            
            smach.StateMachine.add(
                'say_calibring',
                text_to_say(SAY_CALIBRING,wait=True),
                transitions={'succeeded': 'Activate_Asr', 'aborted': 'Activate_Asr', 'preempted': 'Activate_Asr'})

            smach.StateMachine.add('Activate_Asr',
                    ServiceState('/asr_service',
                        ASRService,
                        request_cb = AsrServerRequestActivate_cb,
                        response_cb = AsrServerRequestActivate_response_cb),
                    transitions={'succeeded':'wait', 'aborted': 'wait', 'preempted': 'preempted'})
        
            smach.StateMachine.add(
                    'wait',
                    waitstate(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 'preempted': 'succeeded'})

        
        
