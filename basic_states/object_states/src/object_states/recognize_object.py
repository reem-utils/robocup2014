#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

@created 15 Apr 2014
@las_modified 29 June 2014
@author: Cristina De Saint Germain
@author: Chang Long Zhu

"""

import rospy
import smach

from operator import attrgetter
from object_states.object_detect_sm import object_detect_sm

from object_recognition_msgs.msg import RecognizedObject, RecognizedObjectArray  
from geometry_msgs.msg import PoseWithCovarianceStamped

from manipulation_states.move_head_form import move_head_form

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class process_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error','object_name', 'object_pose'],
                             output_keys=['standard_error','object_position'])

    def execute(self, userdata):
        object_correct = []
        
        if userdata.object_pose.objects:            
            #Object_correct would be of type RecognizedObject
            object_correct = [objectd for objectd in userdata.object_pose.objects if objectd.type.key==userdata.object_name]
            if object_correct:
                object_aux = object_correct.pop()
                userdata.object_position = object_aux.pose #PoseWithCovarianceStamped
                
                userdata.standard_error="Recognize_object OK" + userdata.standard_error
                return 'succeeded'
            else :
                userdata.standard_error="No Object with that identifier " + userdata.standard_error
                return 'aborted'
        else:
            userdata.standard_error="no objects available"+userdata.standard_error
            userdata.objectd=None
            return 'aborted'


class recognize_object(smach.StateMachine): 
    """
    Executes a SM that look if it can recognize object
    
    It have 2 options:
        if you complete the name, it will return if
            it find this object, and return the object message of it.
        If you don't complete it will return the object with more confidence.
    
    
    Required parameters : 
    No parameters.

    Optional parameters:
            object_name, of the person that you are looking for, it will return
                        aborted if can't find 

    input keys:
            object_name, it's optional of the person we are looking for, it can be the name or ""
    output keys:
            standard_error: inform what is the problem
            object_position, is a message that have ObjectDetection, 
                it will be None if can't find any object
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['object_name'], 
                                 output_keys=['standard_error','object_position'])
        
        with self:
            smach.StateMachine.add(
                                   'Look_down',
                                   move_head_form(head_left_right="center", head_up_down="down"),
                                   transitions={'succeeded': 'detect_object', 
                                                'aborted': 'aborted', 
                                                'preempted': 'preempted'})
            smach.StateMachine.add(
                    'detect_object',
                    object_detect_sm(),
                    transitions={'succeeded': 'process_object', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            
            smach.StateMachine.add(
                    'process_object',
                    process_object(),
                    transitions={'succeeded': 'Look_normal', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            
            smach.StateMachine.add(
                   'Look_normal',
                   move_head_form(head_left_right="center", head_up_down="normal"),
                   transitions={'succeeded': 'succeeded', 
                                'aborted': 'aborted', 
                                'preempted': 'preempted'})




                 


