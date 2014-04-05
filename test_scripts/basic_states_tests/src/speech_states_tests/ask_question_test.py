#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
2 April 2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach
import smach_ros
import actionlib

from speech_states.ask_question import AskQuestionSM

def main():
    rospy.init_node('ask_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

	sm.userdata.tts_text = None
	sm.userdata.grammar_name = 'GrammarTest'

        smach.StateMachine.add(
            'AskQuestionSM',
            AskQuestionSM("Would you like a drink?"),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'asksm_test_introspection', sm, '/ASK_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
