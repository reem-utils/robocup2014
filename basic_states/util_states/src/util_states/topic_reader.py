#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Chang long Zhu
@email: changlongzj@gmail.com
"""
import rospy
import smach
import copy

#from global_common import ''preempted'', aborted, preempted, o1, o2, o3, o4
class topic_reader_state(smach.State):
    
    def __init__(self, topic_name, topic_type, topic_time_out,blocked=False):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_time_out = topic_time_out
        self.blocked=blocked
        self.received_msg=False

    def execute(self, userdata):
        self.received_msg=False
        self.time_init=rospy.get_rostime()
        if (self.blocked):
            try:
                _topic_info = rospy.wait_for_message(self.topic_name, self.topic_type, self.topic_time_out)
                userdata.topic_output_msg = _topic_info
                userdata.standard_error = "Topic Reader : No Error "
                if self.preempt_requested():
                    return 'preempted'
                return 'succeeded'
            except rospy.ROSException:
                userdata.standard_error = "Topic Reader : TimeOut Error"
                userdata.topic_output_msg = ''
                if self.preempt_requested():
                    return 'preempted'
                return 'aborted'
        else :
            self.subs=rospy.Subscriber(self.topic_name,self.topic_type, self.callback_topic)
            
            while not self.received_msg and (rospy.get_rostime().secs - self.time_init.secs) < self.topic_time_out:
                rospy.sleep(0.5)
                if self.preempt_requested():
                    return 'preempted'

            if self.received_msg :
                userdata.topic_output_msg = copy.deepcopy(self.msg_data)
                userdata.standard_error = ''
                return 'succeeded'
            else :
                userdata.standard_error = "Topic Reader : TimeOut Error"
                userdata.topic_output_msg = ''
                return 'aborted'
            
    def callback_topic(self,data):
        rospy.loginfo("-------\n we got this data:")
        rospy.loginfo(data)
        self.msg_data = data
        self.received_msg = True
        self.subs.unregister()

class topic_reader(smach.StateMachine):
    """
    Executes a SM that reads a specified topic.
    
    Required parameters:
    @param topic_name: Name of the topic (e.g. \amcl_pose) 
        @type String: 
    @param topic_type: Class of the topic (e.g. PoseStamped)
        @type Class: 
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
     

    No io_keys.

    Example of usage:
        topic_name = '/sonar_base'
        topic_type = Range
        topic_time_out = 60
        
        smach.StateMachine.add(
            'dummy_state',
            topic_reader(topic_name,topic_type,topic_time_out),
            transitions={'succeeded': 'Printing','preempted':'preempted', 'aborted':'aborted'})
    """
    def __init__(self, topic_name, topic_type, topic_time_out,blocked=False):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
 #       rospy.init_node("Topic_reader")

        with self:
            
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader_state(topic_name, topic_type, topic_time_out,blocked), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
