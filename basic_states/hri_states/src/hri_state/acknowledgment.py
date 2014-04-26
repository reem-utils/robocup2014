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

from manipulation_states.play_motion_sm import play_motion_sm

from gesture_detection_mock.msg import Gesture

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class init_var(smach.State):
    
    def __init__(self,type_movement,text_to_say):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['text_to_say','type_movment'],
                             output_keys=['text_to_say','type_movment'],)
    
        self.type_movement=type_movement
        self.text_to_say=text_to_say
        
    def execute(self, userdata):
        
        
        if   userdata.text_to_say ==None :
            userdata.text_to_say=self.text_to_say
        if  userdata.type_movment == None :
            userdata.type_movment=self.type_movement
        
        return 'succeeded'

class acknowledgment(smach.StateMachine): 
    """
    This state will move the head and say information.
    @Inputs userdata.text_to_say // text_to_say : it's the text that will say
            userdata.type_movment //  type_movment : it can be yes or not
     for defauld is yes, you only have to put the sentence
    output keys:
        standard_error: inform what is the problem
        succeeded : if it was possible
        aborted: if it's not possible
        preempted if some one have say 
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self,type_movement="yes",text_to_say=""):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=['standard_error','gesture_detected'])
        
        self.type_movement=type_movement
        self.text_to_say=text_to_say
        
        with self:
            self.userdata.text_to_say =None
            self.userdata.type_movment = None
            
            smach.StateMachine.add(
                                'INIT_VAR',
                                init_var(self.type_movement,
                                         self,text_to_say),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})    
