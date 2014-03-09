#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""


import rospy
from rospy.core import rospyinfo
import smach
from smach_ros import ServiceState
from face_states.recognize_face import recognize_face
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class look_time(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        temps_actual=rospy.Time.now()
        
        if ((temps_actual.secs-self.time_init)>self.maxtime):
            return 'aborted'
        else :
            return 'succeeded'
        


class searching_person(smach.StateMachine): 
    """
    
    This state machine only can return succeeded, it will try to
    find a face all the time, if it find it will return succeeded.
    It can return aborted if the time live.
    
    
    Required parameters : 
    No parameters.

    Optional parameters:
                    name, of the person that you are looking for
                    max time ?¿ maybe it can be interesting to put a maximum time
    No optional parameters


    input keys:
            name, it's optional of the person we are looking for,
                
    output keys:
            standard_error: inform what is the problem
            face, if it find it will return the face
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,maxtime=300,minConfidence=90):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name'], 
                                 output_keys=['standard_error','face'])
        self.time_init=rospy.Time.now()
        self.maxtime=maxtime
        
        with self:

            # Wait learning_time, that the robot will be learning the face
            smach.StateMachine.add(
                                'recognize_face',
                                recognize_face(minConfidence),
                                transitions={'succeeded': 'succeeded', 'aborted': 'look_time', 
                                'preempted': 'preempted'})
            
            # i look the if the time is OK
            smach.StateMachine.add(
                    'look_time',
                    look_time(),
                    transitions={'succeeded': 'recognize_face', 'aborted': 'aborted', 
                    'preempted': 'preempted'})
            




                 



