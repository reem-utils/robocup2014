#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 1 18:30:00 2014

@author: Chang long Zhu
@email: changlongzj@gmail.com
"""
import rospy
import actionlib
import smach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from util_states.topic_reader import topic_reader

# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class prepare_manip_to_pose(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        
class create_move_group_pose_goal(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])

class manip_to_pose(smach.StateMachine):
    rospy.init_node("manip_to_pose")
    def __init__(self, topic_name, topic_type, topic_time_out):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             output_keys=['topic_output_msg', 'standard_error'])
        with self:
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader(topic_name, topic_type, topic_time_out), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})