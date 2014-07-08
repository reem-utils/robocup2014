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
from smach_ros import SimpleActionState, ServiceState

#from robot_inspection_sm import RobotInspectionSM
from basic_functionalities.what_say_sm import WhatSaySM

from speech_states.activate_asr import ActivateASR

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

def main():
    rospy.init_node('what_say_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
	
	sm.userdata.grammar_name="robocup/what_did_you_say_2"

	# Activate the server
        smach.StateMachine.add('ActivateASR',
                    ActivateASR(),
                    transitions={'succeeded': 'what_say_sm', 'aborted': 'aborted', 'preempted': 'preempted'})
            

        smach.StateMachine.add(
            'what_say_sm',
            WhatSaySM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'what_say_test', sm, '/WSI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
