#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import string
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

class GetPoseSubscribe(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
			output_keys=['current_robot_pose', 'standard_error'])

	def execute(self, userdata):
		try:
			pose_current = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 10)
			userdata.current_robot_pose = pose_current.pose
			userdata.standard_error = "OK"
			
			return 'succeeded'
		except rospy.ROSException:
			userdata.standard_error = "get_current_robot_pose : Time_out getting /amcl_pose"
			print 'aborded'
			return 'aborted'
			

class get_current_robot_pose(smach.StateMachine):

	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['current_robot_pose', 'standard_error'])
		rospy.loginfo('GetCurrentPose')
		with self:
			#self.userdata.current_robot_pose = [0.0,0.0,0.0]
			smach.StateMachine.add('GetPose', GetPoseSubscribe(), transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})