#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""


from pal_detection_msgs.msg import FaceDetections
from pal_detection_msgs.srv import RecognizerRequest, Recognizer, RecognizerResponse
import rospy
from rospy.core import rospyinfo
import smach
from smach_ros import ServiceState
from util_states.topic_reader import topic_reader


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class detect_face(smach.StateMachine): 
    """
    Executes a SM that subscribe in a topic and return what faces
    are detect.
    It call a Recognizer Service, it force the start,
    It doesn't close the recognizer

    Required parameters : 
    No parameters.

    Optional parameters:
             minConfidence, is the value to filter the face TODO: i don't know
                             what value for default is ok

    No input keys.       
    output keys:
        standard_error: inform what is the problem
        faces: is a message that have array of face FaceDetection
    No io_keys.

    """
    def __init__(self,minConfidence=90.0):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=['standard_error','faces'])
        
        with self:
            self.userdata.faces = ""
            
            # call request for Recognizer
            @smach.cb_interface(input_keys=[])
            def face_start_detect_request_cb(userdata, request):
                start_request = RecognizerRequest()
                start_request.enabled=True
                start_request.minConfidence=minConfidence
                return start_request
          
            #call request of start recognizer
            smach.StateMachine.add('Start_recognizer',
                               ServiceState('/pal_face/recognizer',
                                            Recognizer,
                                            request_cb = face_start_detect_request_cb,
                                            input_keys = []),
                               transitions={'succeeded':'Read_Topic','aborted' : 'aborted','preempted':'preempted'})
           
                 
            smach.StateMachine.add(
                                'Read_Topic',
                                topic_reader('/pal_face/faces', FaceDetections, 60),
                                remapping={'topic_output_msg': 'faces'},
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
      


