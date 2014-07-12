#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 8 19:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import sys
import actionlib
import rospy
import smach

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from smach_ros.simple_action_state import SimpleActionState
from move_joints_group import move_joints_group
from manipulation_states.move_head import move_head

class prepareData(smach.State):

    def __init__(self, head_left_right=None, head_up_down=None):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted', 'preempted'],
                            input_keys=['head_left_right', 'head_up_down'],
                            output_keys=['move_head_pose'])
        self.head_lr_temp = head_left_right
        self.head_ud_temp = head_up_down
    def execute(self, userdata):
        self.head_lr = self.head_lr_temp if self.head_lr_temp else userdata.head_left_right   
        self.head_ud = self.head_ud_temp if self.head_ud_temp else userdata.head_up_down   
        left_right = 0.0
        up_down = 0.0

        if self.head_lr == "total_left":
            left_right = 1.0
        elif self.head_lr == "mid_left":
            left_right = 0.5
        elif self.head_lr == "total_right":
            left_right = -1.0
        elif self.head_lr == "mid_right":
            left_right = -0.5
        elif self.head_lr == "center":
            left_right = 0.0

        if self.head_ud == "down":
            up_down = 0.5
        elif self.head_ud == "normal":
            up_down = 0.1
        elif self.head_ud == "up":
            up_down = -0.15 # TODO before it was -0.1
            
        userdata.move_head_pose = [left_right, up_down]
        if self.preempt_requested():
            return 'preempted'
        return 'succeeded'
        
class move_head_form(smach.StateMachine):
    
    """
    This SM executes the "move_head" SM from the Manipulation_States.
    It gets a name of a defined hand_pose name, this set of predefined poses are:
        - "total_left" : The robot is looking at the left
        - "mid_left" : The robot is looking at the mid left
        - "total_right" : The robot is looking at the right
        - "mid_right" : The robot is looking at the mid right
        - "center" : The robot is looking at the center
        - "down" : The robot is looking down
        - "normal" : The robot is looking at the normal inclination
        - "up" :
    Parameters:
        @param: head_left_right:
            "total_left", "mid_left", "total_right", "mid_right", "center"
        @param: head_up_down:
             "down", "normal"

    No input_keys
       @key head_left_right
       @key head_up_down
       ,
    No output_keys
        @key move_head_pose
    @Usage:
        smach.StateMachine.add('dummy_state',
                                move_head_form(hand_pose_name="total_left", hand_side="normal"),
                                transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        
    """
    def __init__(self, head_left_right=None, head_up_down=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['head_left_right', 'head_up_down'],
                                    output_keys=['move_head_pose'])
        with self:
            self.userdata.head_left_right=None
            self.userdata.head_up_down=None
            smach.StateMachine.add('PrepareData', 
                                    prepareData(head_left_right,head_up_down),
                                    transitions={'succeeded':'Move_head', 'aborted':'Move_head', 'preempted':'preempted'})
            smach.StateMachine.add('Move_head',
                                    move_head(),
                                    transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})


