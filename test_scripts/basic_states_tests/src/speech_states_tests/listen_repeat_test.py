#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

26 03 2014
"""

import rospy
import smach
import smach_ros
import actionlib

from speech_states.listen_and_repeat import ListenRepeatSM
"""
    This file tests the listen_and_repeat function
    
"""

def main():
    rospy.init_node('listen_repeat_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        sm.userdata.grammar_name = None

        smach.StateMachine.add('ListenRepeatTest',
            ListenRepeatSM("YesNo"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'listen_repeat_test_introspection', sm, '/LISTEN_REPEAT_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
