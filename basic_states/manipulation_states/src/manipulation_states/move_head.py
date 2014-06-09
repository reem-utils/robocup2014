#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 8 19:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import sys
import actionlib
import rospy
import smach

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from smach_ros.simple_action_state import SimpleActionState
from move_joints_group import move_joints_group

# Useful dictionary for reading in an human friendly way errors
traj_error_dict = {}
for name in FollowJointTrajectoryResult.__dict__.keys():
    if not name[:1] == '_':
        code = FollowJointTrajectoryResult.__dict__[name]
        traj_error_dict[code] = name
        
class prepare_move_joints_group(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo('Executing Move head')
        if self.preempt_requested():
            return 'preempted'
        rospy.sleep(1)
        
        return 'succeeded'
class create_move_head_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             input_keys=['move_head_pose'],
                             output_keys=['standard_error', 'move_joint_group','move_joint_poses', 'move_joint_list'],
                             outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo('In create_move_head_goal')
        try:
            userdata.move_joint_group = 'head_controller'
            rospy.loginfo('After setting move_joint_group : head_controller')
            move_joint_list = []
            move_joint_list.append('head_1_joint')
            move_joint_list.append('head_2_joint')
                        
            rospy.loginfo('Joint List is:: ' + str(move_joint_list))
                       
            userdata.move_joint_list = move_joint_list
            userdata.move_joint_poses = userdata.move_head_pose
            userdata.standard_error = "Successful at creating move_head_goal"
            return 'succeeded'
        except ValueError:
            userdata.standard_error = "Error at Move_head SM"
            return 'aborted'
            
        
class move_head(smach.StateMachine):
    
    """
    This SM moves a the HEAD group of joints.
    
    Required parameters: None
    
    Optional parameters: None
    
    Output keys: 
        @key standard_error: Error 
    
    Input keys:
        @key move_head_pose: indicates poses for the joints to reach : 2 joints [head1, head2]
        
        Range: [-1,1] 

    @Usage:
        sm.userdata.move_head_pose = [0.1, 1]        
        smach.StateMachine.add(
            'dummy_state',
            move_head(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['move_head_pose'],
                                    output_keys=['standard_error', 'move_joint_group','move_joint_poses', 'move_joint_list'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_move_head',
                                    prepare_move_joints_group(), 
                                    transitions={'succeeded':'prepare_move_head_goal', 'preempted':'preempted', 'aborted':'aborted'})
            
            # Preparation of the Goal
            smach.StateMachine.add('prepare_move_head_goal',
                                    create_move_head_goal(), 
                                    transitions={'succeeded':'send_move_head_goal', 'preempted':'preempted', 'aborted':'aborted'})
                
            # Send the goal
            smach.StateMachine.add('send_move_head_goal', 
                                   move_joints_group('head_controller'),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})





