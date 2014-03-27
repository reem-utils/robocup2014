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
#from smach_ros import SimpleActionState, ServiceState


from say import text_to_say


class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=[])

    def execute(self, userdata):
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
            transitions={'succeeded': 'SaySM'})

        sm.userdata.tts_text = "Congratulations Sergi"
        sm.userdata.tts_wait_before_speaking = 0
        smach.StateMachine.add(
            'SaySM',
            text_to_say(),
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
