#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed March 5 12:30:00 2014

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
from util_states.topic_reader import topic_reader
from smach_ros.simple_action_state import SimpleActionState
from manip_to_pose import manip_to_pose

class print_userdata(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['standard_error'], outcomes=['succeeded', 'aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo('Userdata Standard Error' + userdata.standard_error)
        return 'succeeded'
def main():
    rospy.loginfo('Manip_to_pose_training INIT')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        sm.userdata.manip_group = 'right_arm'
        
        sm.userdata.manip_goal_pose = Point(0.25, -0.25, 1.2) # right_arm arm_right_tool_link can definitely get here

        sm.userdata.manip_end_link = 'hand_right_grasping_frame'
        
        smach.StateMachine.add(
            'dummy_state',
            manip_to_pose(),
            transitions={'succeeded': 'print','preempted':'preempted', 'aborted':'aborted'})
        smach.StateMachine.add('print', print_userdata(), transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        
    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()