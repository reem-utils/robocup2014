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
    
    def __init__(self, topic_name, topic_type, topic_time_out,blocked=False):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_time_out = topic_time_out
        self.blocked=blocked
        self.msg=False
        self.time_exit=False
        self.time_init=rospy.get_rostime()
    def calul_time(self):
        #if rospy.get_rostime().secs-self.time_init.secs > self.time_out :
         #   self.time_exit=True
         self.time_exit=False
            
        

    def execute(self, userdata):
        if (self.blocked):
            try:
                rospy.loginfo('[TopicReader] In Execute')
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
            self.pub=rospy.Subscriber(self.topic_name,self.topic_type, self.callback)
            
            while not self.msg and not self.time_exit :
                self.calul_time()
                rospy.sleep(0.02)
                if self.preempt_requested():
                    return 'preempted'

            if self.msg :
   
                userdata.topic_output_msg=self.msg_data
                return 'succeeded'
            else :
                
                userdata.standard_error = "Topic Reader : TimeOut Error"
                userdata.topic_output_msg = ''
                return 'aborted'
            
    def callback(self,data):
        self.msg_data=data
        self.msg=True
        self.pub.unregister()
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
        self.blocked=blocked
        with self:
            
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader_state(topic_name, topic_type, topic_time_out,self.blocked), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
