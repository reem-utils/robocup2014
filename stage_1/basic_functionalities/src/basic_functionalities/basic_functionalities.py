#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author:  Cristina De Saint Germain
@email: crsaintc8@gmail.com

26 Feb 2014
"""

import rospy
import smach
import smach_ros
import actionlib

from basic_functionalities_sm import BasicFunctionalitiesSM

def main():
    rospy.init_node('basic_functionalities')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'BasicFunctionalitiesSM',
            BasicFunctionalitiesSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'basic_functionalities_introspection', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
