#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
7/4/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros
import actionlib

from gesture_states.gesture_recognition import GestureRecognition

def main():
    rospy.init_node('gesture_recognition_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        
        sm.userdata.gesture_name = ""

        smach.StateMachine.add(
            'GestureSM',
            GestureRecognition("home"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'gesture_test_introspection', sm, '/GESTURE_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
