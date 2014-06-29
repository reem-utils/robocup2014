#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun June 29 20:00:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com

"""


import rospy
import actionlib
import smach
import smach_ros

from smach_ros.simple_action_state import SimpleActionState

from object_states.search_object import SearchObjectSM
from object_grasping_state.pick_object_sm import pick_object_sm

import time

class prepare_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['object_position'],
                             output_keys=['object_position'])
        
    def execute(self, userdata):
        userdata.object_position = userdata.object_position

        #TODO: Add 0.1 to object_pose z axis
        
        
class object_detection_and_grasping_sm(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 
                                              'aborted', 
                                              'preempted', 
                                              'fail_object_detection',
                                              'fail_object_grasping'], 
                                    input_keys=['object_name', 'time_out_grasp'],
                                    output_keys=['standard_error'])
        
        with self:

            smach.StateMachine.add('Search_Object',
                                   SearchObjectSM(),
                                   transitions={'succeeded':'Grasp_object', 
                                                'aborted':'fail_object_detection', 
                                                'preempted':'preempted'})
            smach.StateMachine.add('Prepare_Grasp',
                                   prepare_grasp(),
                                   transitions={'succeeded':'Grasp_object',
                                                'aborted':'aborted'})
            smach.StateMachine.add('Grasp_object',
                                   pick_object_sm(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'fail_object_grasping',
                                                'preempted':'preempted'})
            