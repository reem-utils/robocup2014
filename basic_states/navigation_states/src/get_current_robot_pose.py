#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import play_motion

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from play_motion.msg import PlayMotionGoal, PlayMotionAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

class GetPoseSubscribe(smach.StateMachine):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
	def execute():
		



class GetCurrentPose(smach.State):

	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted','exit'], output_keys=['current_robot_pose'])
		with self:
			
