#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import sys          #required for arguments

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from play_motion.msg import PlayMotionGoal, PlayMotionAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

class createPlayMotionGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['manip_motion_to_play','manip_time_to_play'],
                             output_keys=['play_motion_sm_goal'])

    def execute(self, userdata):
        play_goal = PlayMotionGoal()
        play_goal.motion_name = userdata.manip_motion_to_play
        play_goal.reach_time.secs = userdata.manip_time_to_play
        userdata.play_motion_sm_goal = play_goal
        
        return 'succeeded'


class play_motion_finish_Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=['standard_error'])
    
    def execute(self, userdata):
        userdata.standard_error = "play_motion_sm : Play_motion error"

        return 'aborted'
        
        
class play_motion_sm(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['manip_motion_to_play','manip_time_to_play'],
                                    output_keys=['standard_error'])
        rospy.loginfo('Play Motion StateMachine')
        with self:
            #Prepare Play Motion Goal
            smach.StateMachine.add('Prepare_play_motion_goal',
                                   createPlayMotionGoal(),
                                   transitions={'succeeded':'Play_Motion_Send', 'aborted':'aborted', 'preempted':'preempted'})
            
            #Send Play motion Goal
            smach.StateMachine.add('Play_Motion_Send', 
                              SimpleActionState('/play_motion', PlayMotionAction, goal_key='play_motion_sm_goal'), 
                              transitions={'succeeded':'succeeded', 'preempted':'play_motion_finish_Error', 'aborted':'play_motion_finish_Error'})
            
            smach.StateMachine.add('play_motion_finish_Error',
                                   play_motion_finish_Error(),
                                   transitions={'succeeded':'aborted', 'aborted':'aborted', 'preempted':'aborted'})
            
            
            
            
            
            