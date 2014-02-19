#!/usr/bin/env python

"""
Author:  Chang
Email: Chang@dsfjakldfa
19 Feb 2014
"""


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

class nav_to_coord(smach.StateMachine):

    """
    Navigate to Coordenates

    This SM navigates to a given coordenates using the parameter: "nav_to_coord_goal"
    This input data should be a (x,y,yaw) point
    
    @input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
    rotation in radians
    @output_keys: standard_error string representing the possible error

    
    """
    def create_nav_goal(self, x, y, yaw):
        """Create a MoveBaseGoal with x, y position and yaw rotation (in radians).
        Returns a MoveBaseGoal"""
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = '/map' # Note: the frame_id must be map
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
         
        # Orientation of the robot is expressed in the yaw value of euler angles
        quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw
        mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
         
        return mb_goal
    
    def __init__(self):
        """
        Constructor for nav_to_coord
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['nav_to_coord_goal'],
                                 output_keys=['standard_error'])

        with self: #Adding states to the container

            nav_goal = self.create_nav_goal(self.userdata.nav_to_coord_goal.x, self.userdata.nav_to_coord_goal.y, self.userdata.nav_to_coord_goal.yaw)
            smach.StateMachine.add('MoveRobot', 
								SimpleActionState(NAVIGATION_TOPIC_NAME, MoveBaseAction, nav_goal), 
								transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            
            # TODO: result callback to fulfill standard_error userdata key


