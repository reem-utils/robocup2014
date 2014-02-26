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
from topic_reader import topic_reader
from sensor_msgs.msg import Range



class PrintUserdataPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['topic_output_msg'])

    def execute(self, userdata):
        rospy.loginfo('Current Info Topic : ' + str(userdata.topic_output_msg))

        return 'succeeded'

def main():
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        
        topic_name = '/sonar_base'
        topic_type = Range
        topic_time_out = 60
        
        smach.StateMachine.add(
            'dummy_state',
            topic_reader(topic_name,topic_type,topic_time_out),
            transitions={'succeeded': 'Printing','preempted':'preempted', 'aborted':'aborted'})
        smach.StateMachine.add(
            'Printing',
            PrintUserdataPose(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()

    
