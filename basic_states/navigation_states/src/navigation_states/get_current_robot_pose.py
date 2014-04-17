#!/usr/bin/env python
"""
Created on Tue Oct 22 12:00:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""

import rospy
import smach
import smach_ros
import actionlib
import string
import math

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

class GetPoseSubscribe(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
			input_keys=['current_robot_pose', 'current_robot_yaw'],
			output_keys=['current_robot_pose', 'current_robot_yaw', 'pose_current', 'standard_error'])

	def execute(self, userdata):
		try:
			pose_current = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 10)
			userdata.current_robot_pose = pose_current.pose

			roll, pitch, userdata.current_robot_yaw = euler_from_quaternion([pose_current.pose.pose.orientation.x,
								pose_current.pose.pose.orientation.y,
								pose_current.pose.pose.orientation.z,
								pose_current.pose.pose.orientation.w])
			userdata.standard_error = "OK"
			userdata.pose_current = pose_current
			
			return 'succeeded'
		except rospy.ROSException:
			userdata.standard_error = "get_current_robot_pose : Time_out getting /amcl_pose"
			return 'aborted'
			

class get_current_robot_pose(smach.StateMachine):
	"""
    Return the actual position and the yaw of the robot

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    Output keys:
    	current_robot_pose: PoseWithCovariance that show the position of the robot 
    	current_robot_yaw: Indicates in radians the yaw of the robot
        standard_error: String that show what kind of error could be happened
    No io_keys.

  
    """
	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
			output_keys=['current_robot_pose', 'current_robot_yaw', 'standard_error', 'pose_current'])
		rospy.loginfo('GetCurrentPose')
		with self:
			#self.userdata.current_robot_pose = [0.0,0.0,0.0]
			smach.StateMachine.add('GetPose', GetPoseSubscribe(), transitions={'succeeded':'succeeded', 
																			'aborted':'aborted', 
																			'preempted':'preempted'})
			
			
			
			
			
			
			