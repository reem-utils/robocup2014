#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
6 April 2014

@author: Cristina De Saint Germain 
"""

import rospy
import smach
import smach_ros
import actionlib

from face_states.ask_name_learn_face import SaveFaceSM
from face_states.drop_faces import drop_faces

def main():
    rospy.init_node('ask_name_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        sm.userdata.name = "robocup"
        sm.userdata.purgeAll = True

        smach.StateMachine.add(
            'dropfaces',
            drop_faces(),
            transitions={'succeeded': 'SaveFaceSM', 'aborted': 'aborted'})


        smach.StateMachine.add(
            'SaveFaceSM',
            SaveFaceSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'saveface_test_introspection', sm, '/SAVEFACE_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()