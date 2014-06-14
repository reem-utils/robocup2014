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

from reem_tabletop_grasping.msg import ObjectManipulationAction, ObjectManipulationActionGoal,ObjectManipulationActionResult, ObjectManipulationGoal
from object_recognition_msgs.msg import ObjectType, RecognizedObject, RecognizedObjectArray

OPERATION_PICK = 1
OPERATION_PLACE = 2
OBJECT_MANIPULATION_TOPIC = '/object_manipulation_server_dual_arms'
GROUP_NAME = 'right_arm_torso'


class Prepare_data(smach.State):
    def __init__(self, pose_to_place):
        smach.State.__init__(self, 
                             outcomes = ['succeeded', 'aborted'], 
                             input_keys = ['pose_to_place'], 
                             output_keys = ['object_place_position'])
        self.object_position = pose_to_place
        
    def execute(self, userdata):
        if not self.object_position and not userdata.pose_to_place:
            rospy.logerr("No Object to Detect")
            return 'aborted'
        #Object Position is a PoseStamped
        userdata.object_place_position = self.object_position if self.object_position else userdata.pose_to_place
                
        return 'succeeded'

class Prepare_Place_Goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succeeded', 'aborted'], 
                             input_keys = ['object_place_position'], 
                             output_keys = ['object_manipulation_goal'])
        
    def execute(self, userdata):
        goal_to_send = ObjectManipulationGoal()
        goal_to_send.operation = ObjectManipulationGoal.PLACE
        goal_to_send.group = GROUP_NAME
        
        goal_to_send.target_pose = userdata.object_place_position
        
        userdata.object_manipulation_goal = goal_to_send
        
        return 'succeeded'        

class place_object_sm(smach.StateMachine):
    def __init__(self, pose_to_place=''):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['pose_to_place'],
                                    output_keys=[])
        with self:
            smach.StateMachine.add("Prepare_data", 
                                   Prepare_data(pose_to_place), 
                                   transitions={'succeeded':'Place_Object_Goal','aborted':'aborted'})
            
            smach.StateMachine.add('Place_Object_Goal',
                                   Prepare_Place_Goal(),
                                   transitions={'succeeded':'Place_Object'})
            
            smach.StateMachine.add('Place_Object', 
                                SimpleActionState(OBJECT_MANIPULATION_TOPIC,
                                                   ObjectManipulationAction,
                                                   goal_key='object_manipulation_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error']), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            

                