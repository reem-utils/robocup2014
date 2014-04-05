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
    def execute(self, userdata):
        rospy.sleep(1)
        return 'succeeded'
        
class create_move_group_joints_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['manip_joint_names','manip_joint_group', 'manip_goal_joint_pose', 'move_it_joint_goal'],
                                    output_keys=['move_it_joint_goal','standard_error'])
    def execute(self, userdata):
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()
        userdata.move_it_joint_goal =  MoveGroupGoal()
        goal_c = Constraints()
        for name, value in zip(userdata.manip_joint_names, userdata.manip_goal_joint_pose):
            joint_c = JointConstraint()
            joint_c.joint_name = name
            joint_c.position = value
            joint_c.tolerance_above = 0.01
            joint_c.tolerance_below = 0.01
            joint_c.weight = 1.0
            goal_c.joint_constraints.append(joint_c)
        
        userdata.move_it_joint_goal.request.goal_constraints.append(goal_c)
        userdata.move_it_joint_goal.request.num_planning_attempts = 5
        userdata.move_it_joint_goal.request.allowed_planning_time = 5.0
        userdata.move_it_joint_goal.planning_options.plan_only = False #False = Plan + Execute ; True = Plan only
        userdata.move_it_joint_goal.planning_options.planning_scene_diff.is_diff = True
        userdata.move_it_joint_goal.request.group_name = userdata.manip_joint_group
        rospy.loginfo('Group Name: ' + userdata.manip_joint_group)
        rospy.loginfo('Joints name: ' + str(userdata.manip_joint_names))
        rospy.loginfo('Joints Values: ' + str(userdata.manip_goal_joint_pose))
        
        
        rospy.loginfo('GOAL TO SEND IS:... ' + str(userdata.move_it_joint_goal))
        return 'succeeded'
        
        
class manip_to_joint_pose(smach.StateMachine):
    """
    Executes a SM that makes the upper body movement of the robot.
    It needs a joint goal and moves the body part to that goal.
    It uses MoveIt! A software for body manipulation. It analyzes the best path to reach to the pose/goal.
        
    Required parameters: None
    
    Optional parameters: None
    
    Input keys:
        @key manip_joint_names: indicates the joints name which we want to move.
        @key manip_joint_group: indicates the joints group name (right_arm, right_arm_torso, right_arm_torso_grasping, 
                                left_arm, left_arm_torso, left_arm_torso_grasping, head, both_arms, both_arms_and_head,
                                right_hand, left_hand)
        @key manip_goal_pose: indicates the pose to reach for joints specified in manip_joint_names
            
         
    Output keys:
        @key standard_error: Error
    """
        
    def __init__(self):
        #rospy.init_node("manip_to_joint_pose")
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['manip_joint_names','manip_joint_group', 'manip_goal_joint_pose'],
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
            
            def move_result_cb(self, error, move_result):
                print(str(move_result.error_code))
                if move_result.error_code.val != 1:
                    rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[move_result.error_code.val]  + "\"")
                    self.standard_error = "manip_to_joint_pose Goal not succeeded: \"" + moveit_error_dict[move_result.error_code.val]  + "\""
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
                                   SimpleActionState('/move_group', MoveGroupAction, 
                                   goal_key='move_it_joint_goal',
                                   exec_timeout=rospy.Duration(20.0), 
                                   result_cb=move_result_cb, 
                                   input_keys=['standard_error'], 
                                   output_keys=['standard_error']),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})





