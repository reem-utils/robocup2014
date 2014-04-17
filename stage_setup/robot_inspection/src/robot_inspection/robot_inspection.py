#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: sampfeiffer
"""

import rospy
import smach
import smach_ros
#from smach_ros import SimpleActionState, ServiceState


from robot_inspection_sm import RobotInspectionSM
from check_dependences import CheckDependencesState

def main():
    rospy.init_node('robot_inspection')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'CheckDepencences',
            CheckDependencesState(),
            transitions={'succeeded': 'RobotInspectionSM', 'aborted': 'aborted'})
        
        smach.StateMachine.add(
            'RobotInspectionSM',
            RobotInspectionSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'robot_inspection_introspection', sm, '/RI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
