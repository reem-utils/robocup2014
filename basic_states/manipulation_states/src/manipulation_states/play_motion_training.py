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
import string

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
from play_motion_sm import play_motion_sm

class PrintUserdataPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['current_robot_pose'])

    def execute(self, userdata):
        rospy.loginfo('Current Pose : ' + str(userdata.current_robot_pose))

        return 'succeeded'

def main():
    rospy.loginfo('Main Get Current Position')
    rospy.init_node('get_current_position')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        sm.userdata.manip_motion_to_play = 'home'
        sm.userdata.manip_time_to_play = 4.0
        smach.StateMachine.add(
            'dummy_state',
            play_motion_sm(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
        

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()

    
