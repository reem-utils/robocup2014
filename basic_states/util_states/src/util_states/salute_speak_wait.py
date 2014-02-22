#!/usr/bin/env python

"""
Author:  Chang Long Zhu Jin
Email: changlongzj@gmail.com
22 Feb 2014
"""


#import rospy
import smach
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from play_motion.msg import PlayMotionGoal, PlayMotionAction


class createNavGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
							input_keys=['nav_to_coord_goal'], output_keys=['navigation_goal'])

    def execute(self, userdata):
        nav_goal = self.create_nav_goal(userdata.nav_to_coord_goal[0], 
									userdata.nav_to_coord_goal[1], 
									userdata.nav_to_coord_goal[2])

        userdata.navigation_goal = nav_goal
        return 'succeeded'
 
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

class salute_speak_wait(smach.StateMachine):
    """
    State Machine which includes 3 different SMs:
    - Speak
    - Salute
    - Wait
    
    @input_keys: wait_time_goal type: float representing seconds
    @output_keys: standard_error string representing the possible error
    """

    
    def __init__(self):
        """
        Constructor for salute_speak_wait.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['wait_time_goal'],
                                 output_keys=['standard_error'])

        with self: 
            
            # Concurrence State Machine:
            # 1. Salute
            # 2. Speak
            
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                    default_outcome='succeeded',
                                    outcome_map={'succeeded': {'Salute': 'succeeded', 'Speak': 'succeeded'}})
            
            with sm_conc:
                

                smach.Concurrence.add('Salute',
                                   createNavGoal(),
                                   transitions={'succeeded':'MoveRobot', 'aborted':'aborted'})
                
            
                 smach.Concurrence.add('Speak', 
								SimpleActionState(NAVIGATION_TOPIC_NAME,
                                                   MoveBaseAction,
                                                   goal_key='navigation_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=move_res_cb), 
								transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            

