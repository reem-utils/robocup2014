#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 30 17:00:00 2013

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""

import rospy
import smach
import actionlib
from rospy.core import rospyinfo
from smach_ros import ServiceState

from gesture_detection_mock.msg import Gesture

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class read_topic_gestures(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[''],
         output_keys=['standard_error','gesture_detected'])

    def execute(self, userdata):
        
       # message=FaceDetections()
        message = rospy.wait_for_message('/gesture_detection/gesture', Gesture, 60)
        
        # Check the distance between the robot and the doo
        if message!= None:
            userdata.gesture_detected = message
            userdata.standard_error="Detect Gesture OK"
            return 'succeeded'
        else:
            userdata.gesture_detected=None
            userdata.standard_error="Time live of Gesture Detection"
            return 'aborted'

class detect_object(smach.StateMachine): 
    """
    Executes a SM that subscribe in a topic and return what gestures are detected.
    
    Required parameters : 
    No parameters.

    No optional parameters


    input keys: 
       
    output keys:
        standard_error: inform what is the problem
        gesture_detected: 
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,minConfidence=90.0):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=['standard_error','gesture_detected'])
        
        with self:
           
          
            # Wait learning_time, that the robot will be learning the object
            smach.StateMachine.add(
                                'Read_Topic',
                                read_topic_gestures(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})    
