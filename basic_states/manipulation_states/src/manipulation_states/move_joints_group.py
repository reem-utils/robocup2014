#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 8 18:30:00 2014

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
        rospy.loginfo('Executing Move_Joints_Group_SM')
        rospy.sleep(1)
        return 'succeeded'
class create_move_group_joints_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             input_keys=['move_joint_group','move_joint_poses', 'move_joint_list'],
                             output_keys=['move_joint_goal'],
                             outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo('In create_move_group_joints_goal')
        
        _move_joint_goal = FollowJointTrajectoryGoal()
        _move_joint_goal.trajectory.joint_names = userdata.move_joint_list
        
        jointTrajectoryPoint = JointTrajectoryPoint()
        jointTrajectoryPoint.positions = userdata.move_joint_poses
        
        for x in range(0, len(userdata.move_joint_poses)):
            jointTrajectoryPoint.velocities.append(0.0)
         
        jointTrajectoryPoint.time_from_start = rospy.Duration(2.0)
        
        _move_joint_goal.trajectory.points.append(jointTrajectoryPoint)
        
        for joint in _move_joint_goal.trajectory.joint_names:
            goal_tol = JointTolerance()
            goal_tol.name = joint
            goal_tol.position = 5.0
            goal_tol.velocity = 5.0
            goal_tol.acceleration = 5.0
            _move_joint_goal.goal_tolerance.append(goal_tol)
        _move_joint_goal.goal_time_tolerance = rospy.Duration(2.0)
        _move_joint_goal.trajectory.header.stamp = rospy.Time.now()
        
        rospy.loginfo('Move_Joint_GOAL: ' + str(_move_joint_goal))
        userdata.move_joint_goal = _move_joint_goal
        return 'succeeded'
        
class move_joints_group(smach.StateMachine):
    
    """
    This SM moves a group of joints from a group controller.


    Required parameters: 
        @param move_joint_group_in: indicates the controller associated with the joints 
    
    Optional parameters: None
    
    Input keys:
        @key move_joint_group: indicates the controller associated with the joints
        @key move_joint_list: indicates the joints to control/move
        @key move_joint_poses: indicates the pose/s for each joint              
         
    Output keys:
        @key standard_error: Error
    """
    def __init__(self, move_joint_group_in):
        rospy.init_node("move_joints_group_node")
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['move_joint_group','move_joint_poses', 'move_joint_list'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_move_joints_group',
                                    prepare_move_joints_group(), 
                                    transitions={'succeeded':'prepare_move_joints_goal', 'preempted':'preempted', 'aborted':'aborted'})
            # Preparation of the Goal
            smach.StateMachine.add('prepare_move_joints_goal',
                                    create_move_group_joints_goal(), 
                                    transitions={'succeeded':'send_move_joints_goal', 'preempted':'preempted', 'aborted':'aborted'})
            
            def move_result_cb(self, error, move_result):
                print('MOVE_RESULT_CB: ' + str(move_result))
                if move_result.error_code != 0:
                    rospy.logwarn("Goal not succeeded: \"" + traj_error_dict[move_result.error_code]  + "\"")
                    self.standard_error = "move_to_joint_pose Goal not succeeded: \"" + traj_error_dict[move_result.error_code]  + "\""
                    return 'aborted'
                elif move_result.error_code == 0:
                    rospy.loginfo("Goal achieved.")
                    self.standard_error = "move_to_joint_pose succeeded!"
                    return 'succeeded'
                else:
                    rospy.logerr("move_to_joint_pose : Couldn't get result, something went wrong, the goal probably timed out.")
                    return 'preempted'
            # Send the goal
            smach.StateMachine.add('send_move_joints_goal', 
                                   SimpleActionState('/'+move_joint_group_in+'/follow_joint_trajectory', FollowJointTrajectoryAction, 
                                                     goal_key='move_joint_goal',
                                                     exec_timeout=rospy.Duration(20.0), 
                                                     result_cb=move_result_cb, 
                                                     input_keys=['standard_error'], 
                                                     output_keys=['standard_error']),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
