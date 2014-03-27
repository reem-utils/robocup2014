#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach
import smach_ros
import actionlib

from speech_states.say import text_to_say

def main():
    rospy.init_node('say_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

	sm.userdata.tts_text = None
	sm.userdata.tts_wait_before_speaking = None
	sm.userdata.tts_lang = 'en_US'

        smach.StateMachine.add(
            'SaySM',
            text_to_say("Hello!"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'saysm_test_introspection', sm, '/SAY_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
