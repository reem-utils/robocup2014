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
#from smach_ros import SimpleActionState, ServiceState

from super_state_machine import HelloWorldStateMachine


class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=[])

    def execute(self, userdata):
        print "Dummy state to launch real State Machine"
        rospy.sleep(1) # in seconds

        return 'succeeded'


def main():
    rospy.init_node('sm_example_sm_pkg')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        # Using this state to wait and to initialize stuff if necessary (fill up input/output keys for example)
        smach.StateMachine.add(
            'dummy_state',
            DummyStateMachine(),
            transitions={'succeeded': 'HelloWorldStateMachine'})

        smach.StateMachine.add(
            'HelloWorldStateMachine',
            HelloWorldStateMachine(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'sm_example_sm_pkg_introspection', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
