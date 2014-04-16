#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
15 Apr 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach

from operator import attrgetter
from object_grasping_states.detect_object_sm import detect_object

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class proces_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error','objects','object_name', 'objectd'],
                             output_keys=['standard_error','objectd'])

    def execute(self, userdata):
        
        rospy.logwarn(userdata.objects)
        # first of all i look if it's some faces
        if userdata.objects.objects:
            # i look in what option we are, if we are looking for a name o no
            rospy.logerr(">"+ userdata.object_name +"<")
            
            if userdata.object_name!="":
                userdata.objectd=[objectd for objectd in userdata.objects.objects if objectd.object_name.data==userdata.object_name]
                if userdata.objectd:
                    userdata.objectd=userdata.objectd.pop()
                    userdata.standard_error="Recognize_object_Name OK"+userdata.standard_error
                    rospy.logwarn("OK")
                    return 'succeeded'
                else :
                    userdata.standard_error="Recognize:= Any object with that name"+userdata.standard_error
                    rospy.logwarn("No object with that name")
                    return 'aborted'
            # if we are no looking for a face we will organize
            else:
                # i want to take the best face confidence    
                userdata.objects.objectm.sort(cmp=None, key=attrgetter('confidence'), reverse=True)
                userdata.objectd=userdata.objects.objectm[0]
                userdata.standard_error="Recognize_object_Normal OK"+userdata.standard_error
                return 'succeeded'
        else:
            userdata.standard_error="no objects available"+userdata.standard_error
            userdata.objectd=None
            rospy.logwarn("No objects available")
            return 'aborted'


class recognize_object(smach.StateMachine): 
    """
    Executes a SM that look if it can recognize object
    
    It have 2 options:
        if you complete the name, it will return if
            it find this face, and return the face message of it.
        If you don't complete it will return the face with more confidence.
    
    
    Required parameters : 
    No parameters.

    Optional parameters:
            object_name, of the person that you are looking for, it will return
                        aborted if can't find 

    input keys:
            object_name, it's optional of the person we are looking for, it can be the name or ""
    output keys:
            standard_error: inform what is the problem
            objectd, is a message that have ObjectDetection, 
                it will be None if can't find any faces
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,minConfidence=90):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['object_name'], 
                                 output_keys=['standard_error','objectd'])
        
        with self:

            smach.StateMachine.add(
                    'detect_object',
                    detect_object(minConfidence),
                    transitions={'succeeded': 'proces_object', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            
            smach.StateMachine.add(
                    'proces_object',
                    proces_object(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            




                 


