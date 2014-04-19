#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: sampfeiffer
"""

import rospy
import smach
import smach_ros
import actionlib
from smach_ros import SimpleActionState, ServiceState

#from robot_inspection_sm import RobotInspectionSM
from basic_functionalities.avoid_that_sm import Avoid_That
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=[])

    def execute(self, userdata):
        print "Test state of Avoid That"
        #rospy.sleep(1) # in seconds

        return 'succeeded'

class Avoid_That_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
	print 'info of aborted Avoid That'
        print FAIL + str(userdata.standard_error) + ENDC
        return 'aborted'

def main():
    rospy.init_node('avoid_that_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        # Using this state to wait and to initialize stuff if necessary (fill up input/output keys for example)
        smach.StateMachine.add(
            'dummy_state',
            DummyStateMachine(),
            transitions={'succeeded': 'avoid_that_sm'})

        smach.StateMachine.add(
            'avoid_that_sm',
            Avoid_That(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted_info'})
        smach.StateMachine.add(
            'aborted_info',
            Avoid_That_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'avoid_that_test', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
