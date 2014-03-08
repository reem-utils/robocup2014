#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 8 13:30:00 2014

@author: Chang long Zhu
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
from manip_to_joint_pose import manip_to_joint_pose


def main():
    rospy.loginfo('Manip_to_pose_training INIT')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        sm.userdata.manip_joint_group = 'right_arm_torso'
        joint_list_right_arm_straight_down = [-5.85101288255e-05, -0.00100779755239, 9.26099389043e-05,
                                          0.000257664105577, -1.55489239528e-06, -0.00244347294573,
                                          -2.55958709623e-05]
        joint_list_right_arm_shake_hand_pose = [0.376906673976, 0.114372113957, -0.198407737748,
                                            1.36616457377, 0.970099953413, 0.108292227188,
                                            -0.822999433641]
        sm.userdata.manip_goal_joint_pose = joint_list_right_arm_straight_down

        sm.userdata.manip_joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                   'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                   'arm_right_7_joint']
        
        smach.StateMachine.add(
            'dummy_state',
            manip_to_joint_pose(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()