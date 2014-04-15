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
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
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
        #play_goal.reach_time.secs = userdata.manip_time_to_play
        play_goal.skip_planning = false
	userdata.play_motion_sm_goal = play_goal
        
        return 'succeeded'   
    

class prepareData(smach.State):
    
    def __init__(self, motion, time):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['manip_motion_to_play','manip_time_to_play'], output_keys=['manip_motion_to_play','manip_time_to_play'])
        self.motion = motion
        self.time = time
        
    def execute(self, userdata):
           
        if not self.motion and not userdata.manip_motion_to_play:
            rospy.logerr("Motion isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.manip_motion_to_play = self.motion if self.motion else userdata.manip_motion_to_play   
        userdata.manip_time_to_play = self.time if self.time else userdata.manip_time_to_play 
       
        return 'succeeded'
    
class play_motion_sm(smach.StateMachine):
    """
        This is the play_motion_sm. This SM executes the Play_motion functionality, 
        which specifies the position of the different joints in the time.
        
        Required parameters:
        No parameters.
    
        Optional parameters:
        No optional parameters
    
    
        Input Keys:
        @key manip_motion_to_play: specifies the motion (from a predefined set of motions in a .yaml file)
        @key manip_time_to_play: specifies the time to reach the motion. If exceeded an error is produced.

        Output Keys:
        @key standard_error: Specifies an error output occurred in the SM.
        No io_keys.
    
        Nothing must be taken into account to use this SM.
    """
    def __init__(self, motion = None, time = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['manip_motion_to_play','manip_time_to_play'],
                                    output_keys=['standard_error'])
        rospy.loginfo('Play Motion StateMachine')
        with self:
            
            #Prepare Play Motion Goal    
            smach.StateMachine.add('PrepareData',
                                   prepareData(motion, time),
                                   transitions={'succeeded':'Prepare_play_motion_goal', 'aborted':'aborted'})
            
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
                        #return 'succeeded'
                    elif result.error_code == -7:
                        userdata.standard_error = "Play Motion not available: Planner Offline"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -8:
                        userdata.standard_error = "Play Motion not available: No Plan Found"
                        rospy.loginfo(userdata.standard_error)
                    elif result.error_code == -42:
                        userdata.standard_error = "Play Motion not available: Other Error"
                        rospy.loginfo(userdata.standard_error)    
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
            
