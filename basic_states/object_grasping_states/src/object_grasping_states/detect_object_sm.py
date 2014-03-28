#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""

import rospy
import smach
import actionlib
from rospy.core import rospyinfo
from smach_ros import ServiceState

from grasping_mock.msg import ObjectDetection
from grasping_mock.srv import Recognizer, RecognizerRequest, RecognizerResponse

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class read_topic_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error'],
         output_keys=['standard_error','faces'])

    def execute(self, userdata):
        
       # message=FaceDetections()
        message = rospy.wait_for_message('/object_detect/recognizer', ObjectDetection, 60)
        
        # Check the distance between the robot and the doo
        if message!= None:
            userdata.object = message
            userdata.standard_error="Detect_face OK"
            return 'succeeded'
        else:
            userdata.object=None
            userdata.standard_error="Time live of Object Detection"
            return 'aborted'

class detect_object(smach.StateMachine): 
    """
    Executes a SM that subscribe in a topic and return what objects are detected.
    It call a Recognizer Service, it forces the start,
    It doesn't close the recognizer


    Required parameters : 
    No parameters.

    Optional parameters:
             minConfidence, is the value to filter the face TODO: i don't know
                             what value for default is ok
    No optional parameters


    input keys: 
       
    output keys:
        standard_error: inform what is the problem
        object: ObjectDetectionMessage
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,minConfidence=90.0):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=['standard_error','object'])
        
        with self:
            # call request for Recognizer
            @smach.cb_interface(input_keys=[])
            def object_start_detect_request_cb(userdata, request):
                start_request = RecognizerRequest()
                start_request.enabled=True
                start_request.minConfidence=minConfidence
                return start_request
          
            #call request of start recognizer
            smach.StateMachine.add('Start_recognizer',
                               ServiceState('/object_detect/recognizer',
                                            Recognizer,
                                            request_cb = object_start_detect_request_cb,
                                            input_keys = []),
                               transitions={'succeeded':'Read_Topic','aborted' : 'aborted','preempted':'preempted'})
            # Wait learning_time, that the robot will be learning the object
            smach.StateMachine.add(
                                'Read_Topic',
                                read_topic_objects(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})    
