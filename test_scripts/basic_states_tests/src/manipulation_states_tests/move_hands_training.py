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
from manipulation_states.move_hands_form import move_hands_form

def main():
    rospy.loginfo('Move_joint_pose_training INIT')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add('dummy_state',
                                move_hands_form(hand_pose_name="grasp", hand_side="right"),
                                transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        
        #smach.StateMachine.add(
        #    'dummy_state',
        #    move_hands(move_hand_side_out),
        #    transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()