#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 6 17:40:00 2014

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""

import smach
import rospy

import smach_ros
from smach_ros import SimpleActionState, ServiceState

from reem_tabletop_grasping.msg import ObjectManipulationAction, ObjectManipulationActionGoal,ObjectManipulationActionResult
from object_recognition_msgs.msg import ObjectType, RecognizedObject, RecognizedObjectArray

class Prepare_data(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succeeded', 'aborted'], 
                             input_keys = [], 
                             output_keys = [])
        
    def execute(self, userdata):
        return 'succeeded'
        

class pick_object_sm(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['object_detection'],
                                    output_keys=[])
        with self:
            smach.StateMachine.add("Prepare_data", 
                                   Prepare_data(), 
                                   transitions={'succeeded':'succeeded','aborted':'aborted'})
            
            

                