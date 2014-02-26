#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author:  Cristina De Saint Germain
@email: crsaintc8@gmail.com
"""

import rospy
import smach
import smach_ros
import actionlib

from basic_funcionalities_sm import BasicFuncionalitiesSM

def main():
    rospy.init_node('basic_funcionalities')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'BasicFuncionalitiesSM',
            BasicFUncionalitiesSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'basic_funcionalities_introspection', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
