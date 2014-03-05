#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""

import rospy
import smach
import smach_ros
import actionlib
#from smach_ros import SimpleActionState, ServiceState
from face_states.learn_face import learn_face

class waitstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change learn_face"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(30)
        return 'succeeded'
    
def main():
    rospy.init_node('learn_face')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        smach.StateMachine.add(
            'learn_face',
            learn_face(),
            transitions={'succeeded' : 'succeeded'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'learn_face_introspection', sm, '/RI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
