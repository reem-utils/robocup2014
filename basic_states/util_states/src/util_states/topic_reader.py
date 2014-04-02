#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Chang long Zhu
@email: changlongzj@gmail.com
"""
import rospy
import smach

#from global_common import ''preempted'', aborted, preempted, o1, o2, o3, o4
class topic_reader_state(smach.State):
    
    def __init__(self, topic_name, topic_type, topic_time_out):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_time_out = topic_time_out
    def execute(self, userdata):
        
        try:
            rospy.loginfo('[TopicReader] In Execute')
            _topic_info = rospy.wait_for_message(self.topic_name, self.topic_type, self.topic_time_out)
            userdata.topic_output_msg = _topic_info
            userdata.standard_error = "Topic Reader : No Error "
            return 'succeeded'
        except rospy.ROSException:
            userdata.standard_error = "Topic Reader : TimeOut Error"
            userdata.topic_output_msg = ''
            return 'aborted'
        
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
    def __init__(self, topic_name, topic_type, topic_time_out):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
 #       rospy.init_node("Topic_reader")
        with self:
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader_state(topic_name, topic_type, topic_time_out), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
