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
from smach_ros.simple_action_state import SimpleActionState

# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class prepare_manip_to_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo('Executing Prepare_manip_to_pose')
        rospy.sleep(1);
        return 'succeeded'
        
        
class create_move_group_pose_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['manip_group', 'manip_goal_pose', 'manip_end_link', 'move_it_goal'],
                                    output_keys=['move_it_goal','standard_error'])
    def execute(self, userdata):
        # Prepare the position
        goal_pose = Pose()
        goal_pose.position = userdata.manip_goal_pose
        # Set the rotation of the tool link, all 0 means for the right hand palm looking left straight
        # roll = rotation over X, pitch = rotation over Y, yaw = rotation over Z
        quat = quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch, yaw
        goal_pose.orientation = Quaternion(*quat.tolist())
        
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()
        
        userdata.move_it_goal = MoveGroupGoal()
        goal_c = Constraints()
        position_c = PositionConstraint()
        position_c.header = header
        
        if userdata.manip_end_link != None: # For some groups the end_link_name can be deduced, but better add it manually
            position_c.link_name = userdata.manip_end_link
            
        # how big is the area where the end effector can be
        position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) 
        position_c.constraint_region.primitive_poses.append(goal_pose)
        
        position_c.weight = 1.0
        
        goal_c.position_constraints.append(position_c)
        
        orientation_c = OrientationConstraint()
        orientation_c.header = header
        if userdata.manip_end_link != None:
            orientation_c.link_name = userdata.manip_end_link
        orientation_c.orientation = goal_pose.orientation
        
        # Tolerances, MoveIt! by default uses 0.001 which may be too low sometimes
        orientation_c.absolute_x_axis_tolerance = 0.01
        orientation_c.absolute_y_axis_tolerance = 0.01
        orientation_c.absolute_z_axis_tolerance = 0.01
        
        orientation_c.weight = 1.0
        
        goal_c.orientation_constraints.append(orientation_c)
        
        userdata.move_it_goal.request.goal_constraints.append(goal_c)
        userdata.move_it_goal.request.num_planning_attempts = 5 # The number of times this plan is to be computed. Shortest solution will be reported.
        userdata.move_it_goal.request.allowed_planning_time = 5.0
        userdata.move_it_goal.planning_options.plan_only = False # True: Plan-Only..... False : Plan + execute
        userdata.move_it_goal.planning_options.planning_scene_diff.is_diff = True # Necessary
        userdata.move_it_goal.request.group_name = userdata.manip_group
        
        return 'succeeded'
        
class manip_to_pose(smach.StateMachine):
    """
    Executes a SM that makes the upper body movement of the robot.
    It needs a 3D pose goal and moves the body part to that goal.
    It uses MoveIt! A software for body manipulation. It analyzes the best path to reach to the pose/goal.    
    Required parameters: None
    
    Optional parameters: None
    
    Input keys:
        manip_group: indicates the group which we want to move. It can be these different groups:
            both_arms, both_arms_torso, 
            left_arm, left_arm_torso, left_hand, --> End effector/link : hand_left_grasping_frame 
            right_arm, right_arm_torso, right_hand --> End effector/link : hand_right_grasping_frame
            
        manip_goal_pose: indicates the pose to reach for the group specified in manip_group, 3D pose
            type: Point() --> x,y,z
         
    Output keys:
        standard_error: Error
        
    """
    
    def __init__(self):
        rospy.init_node("manip_to_pose")
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['manip_group', 'manip_goal_pose', 'manip_end_link'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_manip_to_pose',
                                    prepare_manip_to_pose(), 
                                    transitions={'succeeded':'prepare_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            # Preparation of the Goal
            smach.StateMachine.add('prepare_move_goal',
                                    create_move_group_pose_goal(), 
                                    transitions={'succeeded':'send_move_goal', 'preempted':'preempted', 'aborted':'aborted'})
            
            def move_result_cb(self, error, move_result):
                print(str(move_result.error_code))
                if move_result.error_code.val != 1:
                    rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[move_result.error_code.val]  + "\"")
                    self.standard_error = "manip_to_pose Goal not succeeded: \"" + moveit_error_dict[move_result.error_code.val]  + "\""
                    return 'aborted'
                elif move_result.error_code.val == 1:
                    rospy.loginfo("Goal achieved.")
                    self.standard_error = "manip_to_pose succeeded!"
                    return 'succeeded'
                else:
                    rospy.logerr("manip_to_pose : Couldn't get result, something went wrong, the goal probably timed out.")
                    return 'preempted'
                
            self.userdata.standard_error = ' '
            # Send the goal
            smach.StateMachine.add('send_move_goal', 
                                   SimpleActionState('/move_group', MoveGroupAction, 
                                   goal_key='move_it_goal', 
                                   exec_timeout=rospy.Duration(10.0), 
                                   result_cb=move_result_cb, 
                                   input_keys=['standard_error'], 
                                   output_keys=['standard_error']),
                                   
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
            
            
            
            
            