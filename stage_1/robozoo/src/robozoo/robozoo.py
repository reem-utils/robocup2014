#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author:  Cristina De Saint Germain
@email: crsaintc8@gmail.com

@author:  Sergi Xavier Ubach Pallàs
@email: sxubach@gmail.com

12 Mar 2014
"""

import rospy
import smach
import smach_ros
import actionlib

from check_dependences import CheckDependencesState
from robozoo_sm import RoboZooSM


def main():
    rospy.init_node('robozoo')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
        smach.StateMachine.add(
            'CheckDepencences',
            CheckDependencesState(),
            transitions={'succeeded': 'RoboZooSM', 'aborted': 'aborted'}) 
   
        smach.StateMachine.add(
            'RoboZooSM',
            RoboZooSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'robozoo', sm, '/RZ_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
