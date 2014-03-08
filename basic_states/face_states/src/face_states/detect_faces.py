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


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class read_topic_faces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error'],
         output_keys=['standard_error','faces'])

    def execute(self, userdata):
        
       # message=FaceDetections()
        message = rospy.wait_for_message('/pal_face/recognizer', FaceDetections, 60)
        
        # Check the distance between the robot and the doo
        if message!= None:
            userdata.faces=message
            print ("hello_world")
            userdata.standard_error="Detect_face OK"
            return 'succeeded'
        else:
            userdata.faces=None
            userdata.standard_error="Time live of Face Detection"
            return 'aborted'

class detect_face(smach.StateMachine): 
    """
    Executes a SM that does the process off enrollment.
    It call a Recognizer Service, if force the start,
    It doesn't close the secognizer


    Required parameters : 
    No parameters.

    Optional parameters: learning_time, by default is 5 seconds
    No optional parameters


    input keys: minConfidence, is the value to filter the face TODO:
    output keys: standard_error: inform what is the problem
                faces: is a message that have array of face FaceDetections
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['minCofidence','standard_error'], 
                                 output_keys=['standard_error','faces'])
        
        with self:
            # call request for StardEnrollment
            @smach.cb_interface(input_keys=['minCofidence'])
            def face_start_detect_request_cb(userdata, request):
                start_request = RecognizerRequest()
                start_request.enabled=True
                start_request.minConfidence=userdata.minCofidence
                return start_request
          
            #call request of start recognizer
            smach.StateMachine.add('Start_recognizer',
                               ServiceState('/pal_face/recognizer',
                                            Recognizer,
                                            request_cb = face_start_detect_request_cb,
                                            input_keys = ['minCofidence']),
                               transitions={'succeeded':'Read_Topic','aborted' : 'aborted','preempted':'preempted'})
            # Wait learning_time, that the robot will be learning the face
            smach.StateMachine.add(
                                'Read_Topic',
                                read_topic_faces(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
      
           
        # Go to the init door




                 



