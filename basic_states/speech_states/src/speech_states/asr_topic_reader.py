#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldu
"""
import rospy
import smach
import copy

from pal_interaction_msgs.msg import ASREvent
topic_name='/asr_event'

class topic_reader_state(smach.State):
    
    def __init__(self,topic_time_out):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        self.topic_name = topic_name
        self.topic_time_out = topic_time_out
        print 'Topic reader ASR Init'
    def execute(self, userdata):
        self.time_init=rospy.get_rostime()
        self.msg_data=ASREvent()
        self.subs=rospy.Subscriber(topic_name, ASREvent, self.callback_topic)
        
<<<<<<< HEAD
         # USAR ENUM DE EL TIPO DE EVENT_ID, NO EL NUMERO
        while not self.msg_data.event_id == ASREvent.EVENT_RECOGNIZED_UTT and (rospy.get_rostime().secs - self.time_init.secs) < self.topic_time_out:
            if self.msg_data.recognized_utterance.text =='' :##################k
                print "self.msg_data is: " + str(self.msg_data)
=======
        while not self.msg_data.event_id == ASREvent.EVENT_RECOGNIZED_UTT and (rospy.get_rostime().secs - self.time_init.secs) < self.topic_time_out:
>>>>>>> 1ae5fb70c17987324e8e76e7ccd8f49f4a459682
            rospy.sleep(0.3)
            if self.preempt_requested():
                return 'preempted'

        self.subs.unregister()
<<<<<<< HEAD
        if self.msg_data.event_id == ASREvent.EVENT_RECOGNIZED_UTT :
=======
        if self.msg_data.event_id == ASREvent.EVENT_RECOGNIZED_UTT:
>>>>>>> 1ae5fb70c17987324e8e76e7ccd8f49f4a459682
            userdata.topic_output_msg = copy.deepcopy(self.msg_data)
            userdata.standard_error = ''
            return 'succeeded'
        
        else :
            #time out
            userdata.standard_error = "Topic Reader : TimeOut Error"
            userdata.topic_output_msg = ''
            return 'aborted'
            
    def callback_topic(self,data):
        if data.event_id == ASREvent.EVENT_RECOGNIZED_UTT:
            self.msg_data = data
        

class topic_reader(smach.StateMachine):
    """
    Executes a SM that reads a specified asr topic.

    @param topic_time_out: Number of seconds to 'wait' at maximum to finish the SM
        @type Float: 

    Optional parameters:
    No optional parameters


    No input keys.
    
    Output keys:
    @keyword topic_output_msg: Output of the topic
        @type: topic_type:
    @keyword standard_error: Specifies an error string if occurred.
        @type String: 
    """ 
    
    def __init__(self,topic_time_out):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
 #       rospy.init_node("Topic_reader")

        with self:
            
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader_state(topic_time_out), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
