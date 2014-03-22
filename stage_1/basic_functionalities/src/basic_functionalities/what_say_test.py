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
from what_say_sm import WhatSaySM
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=[])

    def execute(self, userdata):
        print "Test state of What did you say?"
        #rospy.sleep(1) # in seconds

        return 'succeeded'

class Avoid_That_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
	print 'info of aborted What did you say?'
        print FAIL + str(userdata.standard_error) + ENDC
        return 'aborted'
#TODO : check if the function above can be deleted

def main():
    rospy.init_node('what_say_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        # Using this state to wait and to initialize stuff if necessary (fill up input/output keys for example)
        smach.StateMachine.add(
            'dummy_state',
            DummyStateMachine(),
            transitions={'succeeded': 'what_say_sm'})

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
