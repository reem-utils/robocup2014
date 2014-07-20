#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Tue Mar 5 16:18:00 2014

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach

from speech_states.asr_topic_reader import topic_reader_state, topic_reader
from pal_interaction_msgs.msg._ASREvent import ASREvent


class Extraction_cb(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=['topic_output_msg'],
                                output_keys=['asr_userSaid','standard_error',
                                              'asr_userSaid_tags'])
    
    def execute(self, userdata):
       # rospy.loginfo("------------------------------------------------------------------extracting message from topic")
        print('Executing READ ASR')
        userdata.asr_userSaid = userdata.topic_output_msg.recognized_utterance.text
        rospy.loginfo(userdata.topic_output_msg)
        userdata.asr_userSaid_tags = userdata.topic_output_msg.recognized_utterance.tags
        userdata.standard_error = ''
    
        return 'succeeded'  

class check_bucle(smach.State):
    
    def __init__(self,bucle):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=[],
                                output_keys=[])
        self.bucle=bucle
    def execute(self, userdata):
        if self.bucle==False :
            userdata.asr_userSaid = "time out"
            userdata.asr_userSaid_tags = []
            userdata.standard_error = ''
            return 'aborted'
        else :    
            return 'succeeded' 

class ReadASR(smach.StateMachine):
    """
        Read from asr_event and returns what the user has said
        
        @output string asr_userSaid
        @output actiontag[] asr_userSaid_tags
        @ calibrate is hardcode
    
    """

    def __init__(self,Time=30,bucle=True,calibrate=False,Time_calibrate=12):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=[],
                    output_keys=['asr_userSaid', 'standard_error', 'asr_userSaid_tags'])
        
        if calibrate :
            print "Calibrate is true"
            Time=Time_calibrate
        else:
            print "Calibrate is FALSE"
        with self:
            self.userdata.asr_userSaid=None
            self.userdata.standard_error="OK"
            self.userdata.asr_userSaid_tags=None
           
#             # topic reader state
#             smach.StateMachine.add('topicReader',
#                     topic_reader_state(Time,calibrate),
#                     transitions={'succeeded': 'Process', 'aborted': 'check_bucle', 'preempted': 'preempted'})

             # topic reader state
            smach.StateMachine.add('topicReader',
                     topic_reader(Time,calibrate),
                     transitions={'succeeded': 'Process', 'aborted': 'check_bucle', 'preempted': 'preempted'})
            
                        # Process asr_event -> asr_userSaid state       
            smach.StateMachine.add('check_bucle',
                    check_bucle(bucle),
                    transitions={'succeeded': 'topicReader', 'aborted': 'aborted'})
            
            # Process asr_event -> asr_userSaid state       
            smach.StateMachine.add('Process',
                    Extraction_cb(),
                    transitions={'succeeded': 'succeeded'})
            


