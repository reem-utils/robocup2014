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

# Constants
NAVIGATION_TOPIC_NAME = '\move_base'

class nav_to_coord(smach.State):

	"""
	Navigate to Coordenates

	This SM navigates to a given coordenates using the parameter: "nav_to_coord_goal"
	This input data should be a (x,y,z,) point

	"""
	
	def __init__(self):
		"""
		Constructor for nav_to_coord
		@type frame_id: string
		@param frame_id: The frame_id of the pose object. Default is "/base_link"

		@type 
		"""
		#Initialization of the SMACH State machine
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted','exit'], input_keys=['nav_to_coord_goal'])

		self.userdata.xWalk = 0.0

		with self: #Adding states to the container
			#Prepare State, to initialize variables

			#smach.StateMachine.add('Prepare', Prepare(), 
             #                   transitions={'endPrepare':'Concurrence'},
             #                   remapping={'xWalk_in':'xWalk', 'xWalk_out':'xWalk'})
			nav_goal = create_nav_goal(self.userdata.nav_to_coord_goal.x, self.userdata.nav_to_coord_goal.y, 0.0)

			smach.StateMachine.add('MoveRobot', SimpleActionState(NAVIGATION_TOPIC_NAME, MoveBaseAction, nav_goal), transitions={'succeeded':'succeeded', 'aborted':'aborted'})

	def create_nav_goal(x, y, yaw):
	    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
	    Returns a MoveBaseGoal"""
	    mb_goal = MoveBaseGoal()
	    mb_goal.target_pose.header.frame_id = '/map' # Note: the frame_id must be map
	    mb_goal.target_pose.pose.position.x = x
	    mb_goal.target_pose.pose.position.y = y
	    mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
	    
	    # Orientation of the robot is expressed in the yaw value of euler angles
	    angle = radians(yaw) # angles are expressed in radians
	    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
	    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
	    
	    return mb_goal

class Prepare(smach.State):
	    def __init__(self):
	        smach.State.__init__(self, outcomes=['endPrepare'], input_keys=['xWalk_in'], output_keys=['xWalk_out'])

	    def execute(self, userdata):
	        rospy.loginfo('Executing state Prepare')
	        userdata.xWalk_out = 1.0
	        return 'endPrepare'
.