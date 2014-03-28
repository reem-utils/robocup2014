#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun March 8 11:30:00 2014

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


def main():
    rospy.loginfo('Move_joint_pose_training INIT')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        sm.userdata.move_hand_side = 'left'
        move_hand_side_out = 'left_hand_controller'
        sm.userdata.move_hand_pose = [0.1, 0.1, 0.1]  #Full Open
        sm.userdata.move_hand_pose = [5, 5, 5]  #Full Close
        sm.userdata.move_hand_pose = [0.1, 5, 5] #Pregrasp, #Thumb open
        smach.StateMachine.add(
            'dummy_state',
            move_hands(move_hand_side_out),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()