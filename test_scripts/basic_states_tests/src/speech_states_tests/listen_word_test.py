#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
6 April 2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach
import smach_ros
import actionlib

from speech_states.listen_and_check_word import ListenWordSM

def main():
    rospy.init_node('listen_word_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        sm.userdata.word_to_listen = None

        smach.StateMachine.add(
            'ListenWordSM',
            ListenWordSM("Follow me"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'listen_word_test_introspection', sm, '/LISTEN_WORD_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()