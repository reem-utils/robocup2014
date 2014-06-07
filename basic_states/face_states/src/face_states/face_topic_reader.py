#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue June 7 18:00:00 2014

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""
import rospy
import smach
import copy

from pal_detection_msgs.msg import FaceDetections
from pal_detection_msgs.srv import RecognizerRequest, Recognizer, RecognizerResponse
from rospy.core import rospyinfo
from smach_ros import ServiceState

topic_name='/pal_face/faces'

class face_topic_reader_state(smach.State):
    
    def __init__(self,topic_time_out):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        self.topic_name = topic_name
        self.topic_time_out = topic_time_out
        print 'Topic reader Face Init'
    def execute(self, userdata):
        self.time_init=rospy.get_rostime()
        self.msg_data=FaceDetections()
        self.subs=rospy.Subscriber(topic_name, FaceDetections, self.callback_topic)
        

        while len(self.msg_data.faces)==0 and (rospy.get_rostime().secs - self.time_init.secs) < self.topic_time_out:
            rospy.sleep(0.3)
            if self.preempt_requested():
                return 'preempted'

        self.subs.unregister()

        if len(self.msg_data.faces) > 0 :
            userdata.topic_output_msg = copy.deepcopy(self.msg_data)
            userdata.standard_error = ''
            rospy.logwarn('Face Topic Reader: Faces Detected')
            return 'succeeded'
        
        else :
            #time out
            userdata.standard_error = "Topic Reader : TimeOut Error"
            userdata.topic_output_msg = ''
            rospy.logwarn('Face Topic Reader: Time Out Error')
            return 'aborted'
            
    def callback_topic(self,data):
        #If Data is not empty
        if data:
            self.msg_data = data
        

class face_topic_reader(smach.StateMachine):
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
    
    def __init__(self,topic_time_out=20):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])

        with self:
            
            smach.StateMachine.add('Face_topic_reader_state', 
                                   face_topic_reader_state(topic_time_out), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
            
def main():
    rospy.loginfo('Face Topic Reader')
    rospy.init_node('face_topic_reader_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:      
        smach.StateMachine.add(
            'Face_Reader',
            face_topic_reader(20),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()
