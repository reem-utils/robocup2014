#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""

import rospy
import smach

from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalID

from face_states.detect_faces import detect_face
from operator import attrgetter
from util_states.sleeper import Sleeper

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class proces_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error','faces','name','face'],
                             output_keys=['standard_error','face'])

    def execute(self, userdata):
        
        # first of all i look if it's some faces
        if userdata.faces.faces:
            # i look in what option we are, if we are looking for a name o no
            if userdata.name!="":
                userdata.face=[face for face in userdata.faces.faces if face.name==userdata.name]
                if userdata.face:
                    userdata.face=userdata.face.pop()
                    userdata.standard_error="Recognize_face_Name OK"+userdata.standard_error
                    return 'succeeded'
                else :
                    userdata.standard_error="Recognize:= Any face whit that name"+userdata.standard_error
                    return 'aborted'
            # if we are no looking for a face we will organize
            else:
                # i want to take the best face confidence    
                userdata.faces.faces.sort(cmp=None, key=attrgetter('confidence'), reverse=True)
                userdata.face=userdata.faces.faces[0]
                userdata.standard_error="Recognize_face_Normal OK"+userdata.standard_error
                return 'succeeded'
        else:
            userdata.standard_error="no faces available"+userdata.standard_error
            userdata.face=None
            return 'aborted'


class recognize_face(smach.StateMachine): 
    """
    Executes a SM that look if it can recognize faces
    
    It have 2 options:
         if you complete the name, it will return if
            it find this face, and return the face message of it.
        If you don't complete it will return the face with more confidence.
    
    
    Required parameters : 
    No parameters.

    Optional parameters:
                    name, of the person that you are looking for, it will return
                        aborted if can't find 

    input keys:
            Name, it's optional of the person we are looking for, it can be the name or ""
    output keys:
            standard_error: inform what is the problem
            face, is a message that have FaceDetection, 
                it will be None if can't find any faces
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,minConfidence=90):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name'], 
                                 output_keys=['standard_error','face'])
        
        with self:

            self.userdata.face = ""
            
            # extract the database that the robot is finding
            smach.StateMachine.add(
                                'detect_face',
                                detect_face(minConfidence),
                                transitions={'succeeded': 'proces_face', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
            
            # i filter a little bit
            smach.StateMachine.add(
                    'proces_face',
                    proces_face(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            

class CancelNavigation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=[])
    def execute(self, userdata):
        self.face_pub= rospy.Publisher('/move_base/cancel', GoalID)
        self.face_pub.publish(GoalID())
        return 'succeeded'

class recognize_face_concurrent(smach.StateMachine): 
    """
    Executes a SM that look if it can recognize faces
    
    It have 2 options:
         if you complete the name, it will return if
            it find this face, and return the face message of it.
        If you don't complete it will return the face with more confidence.
    
    
    Required parameters : 
    No parameters.

    Optional parameters:
                    name, of the person that you are looking for, it will return
                        aborted if can't find 

    input keys:
            Name, it's optional of the person we are looking for, it can be the name or ""
    output keys:
            standard_error: inform what is the problem
            face, is a message that have FaceDetection, 
                it will be None if can't find any faces
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,minConfidence=90):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name'], 
                                 output_keys=['standard_error','face'])
        
        with self:

            self.userdata.face = ""
            
            # extract the database that the robot is finding
            smach.StateMachine.add(
                                'detect_face',
                                detect_face(minConfidence),
                                transitions={'succeeded': 'Aborting_navigation', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
             #Change succeeded:proces_face
            
            # i filter a little bit
            smach.StateMachine.add(
                    'proces_face',
                    proces_face(),
                    transitions={'succeeded': 'Aborting_navigation', 'aborted': 'time_sleeper', 
                    'preempted': 'preempted'})
            
            smach.StateMachine.add(
                                   'Aborting_navigation',
                                   CancelNavigation(),
                                   transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                                'preempted': 'preempted'})
                               
            smach.StateMachine.add(
                    'time_sleeper',
                    Sleeper(0.5),
                    transitions={'succeeded': 'detect_face', 'aborted': 'detect_face'})
                 



