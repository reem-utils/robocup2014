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
from manipulation_states.move_head import move_head
from manipulation_states.move_head_form import move_head_form


def main():
    rospy.loginfo('Move_joint_pose_training INIT')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        #sm.userdata.move_head_pose = [-1, 0.1]
        #sm.userdata.move_head_pose = [-0.5, 0.1] Mid Right
        #sm.userdata.move_head_pose = [0.5, 0.1] Mid Left
        #sm.userdata.move_head_pose = [1, 0.1] Total Left
        #sm.userdata.move_head_pose = [0.1, 1] Head Down 
        #sm.userdata.move_head_pose = [0.0, 0.1] Head Normal front
        smach.StateMachine.add(
            'move_head',
            move_head_form('mid_right', 'normal'),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('move_head_node')
    main()