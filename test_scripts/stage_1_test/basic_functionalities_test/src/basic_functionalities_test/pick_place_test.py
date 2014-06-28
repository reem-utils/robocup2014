#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""


@author: Roger Boldu
"""

import rospy
import smach
import smach_ros
import actionlib
from smach_ros import SimpleActionState, ServiceState

from basic_functionalities.pick_place_sm import PickPlaceSM

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

class pick_place_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):

        return 'aborted'

def main():
    rospy.init_node('pick_place_node')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
        smach.StateMachine.add(
            'dummy_state',
            DummyStateMachine(),
            transitions={'succeeded': 'pick_place_test'})

        smach.StateMachine.add(
            'pick_place_test',
            PickPlaceSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted_info'})
        smach.StateMachine.add(
            'aborted_info',
            pick_place_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'pick_place_test', sm, '/SM_pick_test')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
