#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Chang long Zhu
@email: changlongzj@gmail.com
"""
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
            
            def play_motion_cb_result(userdata, result_status, result):
                if result.error_code != 1: # 1 == SUCCEEDED
                    print "in not succeeded"
                    if result.error_code == -1: 
                        userdata.standard_error = "Play Motion not available: Motion Not Found"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -2:
                        userdata.standard_error = "Play Motion not available: Infeasible Reach Time"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -3:
                        userdata.standard_error = "Play Motion not available: Controller Busy"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -4:
                        userdata.standard_error = "Play Motion not available: Missing Controller"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -5:
                        userdata.standard_error = "Play Motion not available: Trajectory Error"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -6:
                        
                        userdata.standard_error = "Play Motion not available: Goal Not Reached"
                        
                        rospy.loginfo(userdata.standard_error)
                        
                    elif result.error_code == -7:
                        userdata.standard_error = "Play Motion not available: Planner Offline"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -8:
                        userdata.standard_error = "Play Motion not available: No Plan Found"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -42:
                        userdata.standard_error = "Play Motion not available: Other Error"
                        rospy.loginfo(userdata.standard_error)    
                    print "aborting"
                    return 'aborted'
                else:
                    userdata.standard_error = "Play Motion OK"
                    return 'succeeded'
            #Send Play motion Goal

            smach.StateMachine.add('Play_Motion_Send', 
                                   SimpleActionState('/play_motion', PlayMotionAction, goal_key='play_motion_sm_goal', 
                                                     input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=play_motion_cb_result), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
            