#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 13 19:01:06 2014

@author: Sam Pfeiffer sam.pfeiffer@pal-robotics.com
@author: Chang Long Zhu changlongzj@gmail.com

Snippet of code on how to send a MoveIt! move_group goal to an arm in joint space

Moveit actionserver: /move_group/goal
Type of message: moveit_msgs/MoveGroupGoal

Groups of REEM and their end effectors:

right_arm -> arm_right_tool_link
right_arm_torso -> arm_right_tool_link
right_arm_torso_grasping -> hand_right_grasping_frame

left_arm -> arm_left_tool_link
left_arm_torso -> arm_left_tool_link
left_arm_torso_grasping -> hand_left_grasping_frame

Other groups: both_arms, head, right_hand, left_hand

"""

import rospy
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def create_move_group_pose_goal(goal_pose=Pose(), group="right_arm_torso", end_link_name=None, plan_only=True):
    """ Creates a move_group goal based on pose.
    @arg group string representing the move_group group to use
    @arg end_link_name string representing the ending link to use
    @arg goal_pose Pose() representing the goal pose
    @arg plan_only bool to for only planning or planning and executing
    @returns MoveGroupGoal with the data given on the arguments"""
    
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    # We are filling in the MoveGroupGoal a MotionPlanRequest and a PlanningOptions message
    # http://docs.ros.org/hydro/api/moveit_msgs/html/msg/MotionPlanRequest.html
    # http://docs.ros.org/hydro/api/moveit_msgs/html/msg/PlanningOptions.html
    moveit_goal = MoveGroupGoal()
    goal_c = Constraints()
    position_c = PositionConstraint()
    position_c.header = header
    if end_link_name != None: # For some groups the end_link_name can be deduced, but better add it manually
        position_c.link_name = end_link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # how big is the area where the end effector can be
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 1.0
    goal_c.position_constraints.append(position_c)
    orientation_c = OrientationConstraint()
    orientation_c.header = header
    if end_link_name != None:
        orientation_c.link_name = end_link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01 # Tolerances, MoveIt! by default uses 0.001 which may be too low sometimes
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    moveit_goal.request.goal_constraints.append(goal_c)
    moveit_goal.request.num_planning_attempts = 5 # The number of times this plan is to be computed. Shortest solution will be reported.
    moveit_goal.request.allowed_planning_time = 5.0
    moveit_goal.planning_options.plan_only = plan_only
    moveit_goal.planning_options.planning_scene_diff.is_diff = True # Necessary
    moveit_goal.request.group_name = group
    
    return moveit_goal


if __name__=='__main__':
    #rospy.init_node("moveit_snippet")

    rospy.loginfo("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Creating goal.")
    # Set the position of the tool link, referenced to base_link
    # X = positive in front of the robot, Y = positive to the left of the robot, Z = positive going up
    goal_pose = Pose()
    goal_point = Point(0.25, -0.25, 1.2) # right_arm arm_right_tool_link can definitely get here
    goal_pose.position = goal_point
    # Set the rotation of the tool link, all 0 means for the right hand palm looking left straight
    # roll = rotation over X, pitch = rotation over Y, yaw = rotation over Z
    quat = quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch, yaw
    goal_pose.orientation = Quaternion(*quat.tolist())
    #moveit_goal = create_move_group_pose_goal(goal_pose, group="right_arm", end_link_name="arm_right_tool_link", plan_only=True)
    moveit_goal = create_move_group_pose_goal(goal_pose, group="right_arm", end_link_name="arm_right_tool_link", plan_only=False)
    #moveit_goal = create_move_group_pose_goal(goal_pose, group="right_arm", plan_only=True)
    
    # Available groups:
    # both_arms, both_arms_torso, left_arm, left_arm_torso, left_hand, right_arm, right_arm_torso, right_hand
    rospy.loginfo("Sending goal...")
    moveit_ac.send_goal(moveit_goal)
    rospy.loginfo("Waiting for result...")
    moveit_ac.wait_for_result(rospy.Duration(10.0))
    moveit_result = moveit_ac.get_result()
    
    #rospy.loginfo("Got result:\n" + str(moveit_result)) # Uncomment if you want to see the full result message
    #r = MoveGroupResult()
    if moveit_result != None and moveit_result.error_code.val != 1:
        rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
    elif moveit_result != None:
        rospy.loginfo("Goal achieved.")
    else:
        rospy.logerr("Couldn't get result, something went wrong, the goal probably timed out.")
    
    
