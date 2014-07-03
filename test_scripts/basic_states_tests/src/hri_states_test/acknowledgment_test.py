#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
7/4/2014

@author: Roger Boldu
"""

import rospy
import smach
import smach_ros
import actionlib

from hri_states.acknowledgment import acknowledgment
def main():
    rospy.init_node('acknowledgment_hri')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
	sm.userdata.Skip_planning=False
        smach.StateMachine.add(
            'acknowledgment',
            acknowledgment(type_movement="yes", tts_text="OK, i'm ready, do you wan't to be my friend"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'acknoledgemtn_test', sm, '/acknowledgment')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
