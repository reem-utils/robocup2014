#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 8 11:30:00 2014

@author: Chang long Zhu
@email: changlongzj@gmail.com

Moveit actionserver: /move_group/goal
Type of message: moveit_msgs/MoveGroupGoal

Groups of REEM and their joints:

right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
              'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
              'arm_right_7_joint']

right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                   'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                   'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                   'arm_right_7_joint']
                   
right_arm_torso_grasping = same as right_arm_torso
   
   
left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
           'arm_left_7_joint']
left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                   'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                   'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                   'arm_left_7_joint']
left_arm_torso_grasping = same as left_arm_torso     

Other groups:
head = ['head_1_joint', 'head_2_joint']

both_arms = right_arm_torso + left_arm

both_arms_and_head = right_arm_torso + left_arm + head

right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint']

left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
"""


import rospy
import actionlib
import smach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from smach_ros.simple_action_state import SimpleActionState



# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class prepare_manip_to_joint_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        
class create_move_group_joints_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['manip_group', 'manip_goal_joint_pose', 'manip_end_link', 'move_it_goal'],
                                    output_keys=['move_it_joint_goal','standard_error'])

class manip_to_joint_pose(smach.StateMachine):
    def __init__(self):
        rospy.init_node("manip_to_pose")
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['manip_group', 'manip_goal_pose', 'manip_end_link'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_manip_to_pose',
                                    prepare_manip_to_joint_pose(), 
                                    transitions={'succeeded':'prepare_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            # Preparation of the Goal
            smach.StateMachine.add('prepare_move_goal',
                                    create_move_group_joints_goal(), 
                                    transitions={'succeeded':'send_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            
            def move_result_cb(userdata, error_code):
                if error_code.val != 1:
                    rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[error_code.val]  + "\"")
                    userdata.standard_error = "manip_to_pose Goal not succeeded: \"" + moveit_error_dict[error_code.val]  + "\""
                    return 'aborted'
                elif error_code.val == 1:
                    rospy.loginfo("Goal achieved.")
                    userdata.standard_error = "manip_to_pose succeeded!"
                    return 'succeeded'
                else:
                    rospy.logerr("manip_to_pose : Couldn't get result, something went wrong, the goal probably timed out.")
                    return 'preempted'
                
            # Send the goal
            smach.StateMachine.add('send_move_goal', 
                                   SimpleActionState('/move_group', MoveGroupAction, goal_key='move_it_goal', exec_timeout=10.0, result_cb=move_result_cb, input_keys=['standard_error'], output_keys=['standard_error']),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})





