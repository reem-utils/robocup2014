#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Roger Boldu
"""


import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState
from follow_me_sm import FollowMe 
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



def main():
    rospy.loginfo('follow_me_test')
    rospy.init_node('follow_me_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    with sm:
        sm.userdata.standard_error='OK'
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'prepare_msg',
            FollowMe(),
            transitions={'succeeded':'succeeded','aborted' : 'aborted',
                         'preempted':'preempted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_me_test', sm, '/FME_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
