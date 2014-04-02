#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

28 03 2014
"""

import rospy
import smach
import smach_ros
import actionlib

from speech_states.say_yes_or_no import SayYesOrNoSM
"""
    This file tests the say_yes_or_no function
    
"""

def main():
    rospy.init_node('yes_no_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        sm.userdata.grammar_name = None

        smach.StateMachine.add('YesNoTest',
            SayYesOrNoSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'yes_no_introspection', sm, '/YES_NO_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
