#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created 12:00:00 2013

@author: Roger Boldú
"""


from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest,  StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse
import rospy
from rospy.core import rospyinfo
import smach
from smach_ros import ServiceState
from pal_detection_msgs.srv import RecognizerRequest, Recognizer, RecognizerResponse


from drop_faces import drop_faces
from learn_face import learn_face



# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



class preparedataBase(smach.State):
    
    def __init__(self,name):
        smach.State.__init__(self,input_keys=['name_database'], outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['standard_error','name'])
        self.name = name
    def execute(self, userdata):
        

        if userdata.name_database==None : # this is the database
            userdata.name=self.name
        return 'succeeded'
        

class preparedataName(smach.State):
    
    def __init__(self,name):
        smach.State.__init__(self,input_keys=['name_face'], outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=['name'])
        self.name=name
    def execute(self, userdata):
        
        if userdata.name_face==None : # this is the database
            userdata.name=self.name
        else :
            userdata.name=userdata.name_face
        return 'succeeded'
   
    
    
class new_database_and_learn(smach.StateMachine):
    """
    it creates a database and learn face
    """
    def __init__(self,learning_time=10,minConfidence=80,name_face="unknown",name_database="Robocup"):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name_face','name_database'], 
                                 output_keys=['standard_error'])
        
        self.name_face=name_face
        self.name_database=name_database
        with self:
            self.userdata.name_face=None
            self.userdata.name_database=None
            self.userdata.purgeAll=True
        # Wait learning_time, that the robot will be learning the face
            smach.StateMachine.add(
                                'prepare_data',
                                preparedataBase(self.name_database),
                                transitions={'succeeded': 'drop_face', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
            smach.StateMachine.add(
                                'drop_face',
                                drop_faces(),
                                transitions={'succeeded': 'prepare_face_name', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
            smach.StateMachine.add(
                                'prepare_face_name',
                                preparedataName(name_face),
                                transitions={'succeeded': 'learn_face', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
            smach.StateMachine.add(
                                'learn_face',
                                learn_face(learning_time=learning_time,minConfidence=minConfidence),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
                  
                  
