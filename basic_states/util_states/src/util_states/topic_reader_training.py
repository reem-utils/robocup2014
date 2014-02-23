#!/usr/bin/env python

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

class PrintUserdataPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['topic_info'])

    def execute(self, userdata):
        rospy.loginfo('Current Info Topic : ' + str(userdata.topic_info))

        return 'succeeded'

def main():
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        
        sm.userdata.topic_name = '/amcl_pose'
        sm.userdata.topic_type = PoseWithCovarianceStamped
        sm.userdata.topic_time_out = 60
        
        smach.StateMachine.add(
            'dummy_state',
            topic_reader(),
            transitions={'succeeded': 'Printing','preempted':'preempted', 'aborted':'aborted'})
        smach.StateMachine.add(
            'Printing',
            PrintUserdataPose(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()

    
