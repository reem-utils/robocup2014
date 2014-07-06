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
#from face_states.learn_face import learn_face
from face_states.recognize_face import recognize_face
from face_states.go_find_person import go_find_person
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'




    
    
def main():
    rospy.init_node('go_find_face_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        
        
                # it call the learn state
        smach.StateMachine.add(
            'prepare_face_reconize',
            go_find_person(),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted'})
        


    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'detect_face_introspection', sm, '/DFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
