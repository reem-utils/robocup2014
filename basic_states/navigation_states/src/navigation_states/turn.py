#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22 Febreary 12:00:00 2013

@author: Sergi Xavier Ubach Pallàs
@email: sxubcah@gmail.com

"""

import rospy
import math
import smach
from math import *
from navigation_states.get_current_robot_pose import GetPoseSubscribe
from navigation_states.nav_to_coord import nav_to_coord
from manipulation_states.play_motion_sm import play_motion_sm
from speech_states.say import text_to_say

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


# In this state we will prepare the nav_to_coord_goal needed to turn the yaw we desire
class prepare_nav_to_coord(smach.State):
    def __init__(self,yawPlus,direction):
        smach.State.__init__(self,outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['current_robot_pose','current_robot_yaw','desired_angle','nav_to_coord_goal'],
         output_keys=['nav_to_coord_goal'])
        self.yawPlus = yawPlus
        self.direction = direction
    def execute(self, userdata):
        userdata.nav_to_coord_goal = [0.0,0.0,0.0]
        userdata.nav_to_coord_goal[0] = userdata.current_robot_pose.pose.position.x
        userdata.nav_to_coord_goal[1] = userdata.current_robot_pose.pose.position.y
        
        if self.direction == "left" :
            userdata.nav_to_coord_goal[2] = userdata.current_robot_yaw + math.radians(self.yawPlus)
        else:
            userdata.nav_to_coord_goal[2] = userdata.current_robot_yaw - math.radians(self.yawPlus)
        #print ("-----------------------------------------" + str(userdata.desired_angle) + "----------------------------")
        return "succeeded"
        

        

class turn(smach.StateMachine):
    """
    This state machine receive how many degrees we want to rotate 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters# todo need de mmap

    Input keys: 
        yawPlus: number conatining how many degree we want to rotate
    Output keys:
        standard_error: String that show what kind of error could be happened
    No io_keys.  
    """
    def __init__(self, yawPlus = 0,direction = "left"):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                input_keys=['yaw_plus'],
                                output_keys=['standard_error'])


        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.standard_error=''
            self.userdata.point_to_coord_goal=[0.0,0.0,0.0]
 
            # We get current coodenades of robot in userdata 'current_robot_pose', 'current_robot_yaw', 'pose_current'
            smach.StateMachine.add('get_pose',
               GetPoseSubscribe(),
               transitions={'succeeded': 'prepareNav', 'aborted': 'aborted', 'preempted': 'preempted'})
 
            # Prepares to turn
            smach.StateMachine.add('prepareNav',
               prepare_nav_to_coord(yawPlus,direction),
               transitions={'succeeded': 'turn', 'aborted': 'aborted', 'preempted': 'preempted'})
                
            # Turns
            smach.StateMachine.add('turn',
               nav_to_coord(),
               transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                            




