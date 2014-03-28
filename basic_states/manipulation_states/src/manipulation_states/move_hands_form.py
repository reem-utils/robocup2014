#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri March 28 13:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import actionlib
import smach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from smach_ros.simple_action_state import SimpleActionState
from manipulation_states.move_hands import move_hands

class prepareData(smach.State):

    def __init__(self, hand_pose_name="full_open", hand_side="right"):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted', 'preempted'],
                            output_keys=['move_hand_pose', 'move_hand_side'])
        self.hand_pose = hand_pose_name
        self.hand_side_self = hand_side
    def execute(self, userdata):
        if(_hand_pose == "full_open"):
            userdata.move_hand_pose = [0.1, 0.1, 0.1]
        elif _hand_pose == "grasp":
            sm.userdata.move_hand_pose = [5, 5, 5]
        elif _hand_pose == "pre_grasp":
            sm.userdata.move_hand_pose = [0.1, 5, 5]

        self.move_hand_side_out = self.hand_side_self+'_hand_controller'
        userdata.move_hand_side = self.hand_side_self
        return 'succeeded'


class move_hands_form(smach.StateMachine):
    """
    This SM executes the "move_hands" SM from the Manipulation_States.
    It gets a name of a defined hand_pose name, this set of predefined poses are:
        - "full_open" : Hand is full
        - "grasp" : Hand is completely closed
        - "pre_grasp" : Hand has the thumb open, and two other fingers closed

    """
    def __init__(self, hand_pose_name="full_open", hand_side="right"):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        with self:
            smach.StateMachine.add('PrepareData', 
                                    prepareData(),
                                    outcomes={'succeeded':'Move_Hands', 'aborted':'Move_Hands', 'preempted':'Move_Hands'})
            smach.StateMachine.add('Move_Hands',
                                    move_hands(self.move_hand_side_out),
                                    transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
