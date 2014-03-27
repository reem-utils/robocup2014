#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""

import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState
from speech_states.asr_status import AsrStatus

          
def main():
    rospy.init_node('asr_status_test')
    

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'asr_status_test',
            AsrStatus(),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'asr_status_introspection', sm, '/asrStatus_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
