#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach

from util_states.topic_reader import topic_reader_state
from pal_interaction_msgs.msg._ASREvent import ASREvent


class Extraction_cb(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=['topic_output_msg'],
                                output_keys=['asr_userSaid','standard_error', 'asr_userSaid_tags'])
    
    def execute(self, userdata):
        rospy.loginfo("------------------------------------------------------------------extracting message from topic")
        userdata.asr_userSaid = userdata.topic_output_msg.recognized_utterance.text
        rospy.loginfo(userdata.topic_output_msg)
        userdata.asr_userSaid_tags = userdata.topic_output_msg.recognized_utterance.tags
        userdata.standard_error = ''
    
        return 'succeeded'  

class Topic_Checker(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=['topic_output_msg'],
                                output_keys=['asr_userSaid','standard_error', 'asr_userSaid_tags'])
    
    def execute(self, userdata):
        userdata.asr_userSaid=userdata.topic_output_msg.recognized_utterance.text
        userdata.asr_userSaid_tags=userdata.topic_output_msg.recognized_utterance.tags
        userdata.standard_error="OK"
        rospy.loginfo("------------------------------------------------------------------extracting message from topic")
        if userdata.topic_output_msg.event_id == 2:    
            return 'succeeded'  
        return 'aborted'

class ReadASR(smach.StateMachine):
    """
        Read from asr_event and returns what the user has said
        
        @output string asr_userSaid
        @output actiontag[] asr_userSaid_tags
    
    """

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=[],
                    output_keys=['asr_userSaid', 'standard_error', 'asr_userSaid_tags'])
        
        with self:
            self.userdata.asr_userSaid=None
            self.userdata.standard_error="OK"
            self.userdata.asr_userSaid_tags=None
           
            # topic reader state
            smach.StateMachine.add('topicReader',
                    topic_reader_state('/asr_event', ASREvent, 30),
                    transitions={'succeeded': 'topicChecker', 'aborted': 'aborted', 'preempted': 'preempted'})
            
             # Process asr_event -> asr_userSaid state       
            smach.StateMachine.add('topicChecker',
                    Topic_Checker(),
                    transitions={'succeeded': 'Process', 'aborted':'topicReader'})

            # Process asr_event -> asr_userSaid state       
            smach.StateMachine.add('Process',
                    Extraction_cb(),
                    transitions={'succeeded': 'succeeded'})
            


