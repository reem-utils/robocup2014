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
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        
        return 'succeeded'
        
class move_joints_group(smach.StateMachine):
    
    """
        This SM moves a group of joints from a group controller.
    
    """
    def __init__(self):
        rospy.init_node("manip_to_joint_pose")
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['move_joint_group','move_joint_poses', 'move_joint_list'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_manip_to_pose',
                                    prepare_move_joints_group(), 
                                    transitions={'succeeded':'prepare_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            # Preparation of the Goal
            smach.StateMachine.add('prepare_move_goal',
                                    create_move_group_joints_goal(), 
                                    transitions={'succeeded':'send_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            
            def move_result_cb(self, error, move_result):
                print(str(move_result.error_code))
                if move_result.error_code.val != 1:
                    rospy.logwarn("Goal not succeeded: \"" + traj_error_dict[move_result.error_code.val]  + "\"")
                    self.standard_error = "manip_to_joint_pose Goal not succeeded: \"" + traj_error_dict[move_result.error_code.val]  + "\""
                    return 'aborted'
                elif move_result.error_code.val == 1:
                    rospy.loginfo("Goal achieved.")
                    self.standard_error = "manip_to_joint_pose succeeded!"
                    return 'succeeded'
                else:
                    rospy.logerr("manip_to_joint_pose : Couldn't get result, something went wrong, the goal probably timed out.")
                    return 'preempted'
                
            # Send the goal
            smach.StateMachine.add('send_move_goal', 
                                   SimpleActionState('/move_group', FollowJointTrajectoryAction, 
                                   goal_key='move_it_joint_goal',
                                   exec_timeout=rospy.Duration(20.0), 
                                   result_cb=move_result_cb, 
                                   input_keys=['standard_error'], 
                                   output_keys=['standard_error']),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})





