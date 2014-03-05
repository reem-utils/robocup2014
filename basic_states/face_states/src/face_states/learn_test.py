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
from face_states.learn_face import learn_face
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class prepare_learn_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],output_keys=['name']) #todo: i have to delate de output_key

    def execute(self, userdata):
        rospy.loginfo("prepering learn face")
        userdata.name='pepe'# is the name that it will learn
        return 'succeeded'
class learn_Face_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of aborted learn Face')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'
          
def main():
    rospy.init_node('learn_face_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # it preper the name for learning
        smach.StateMachine.add(
            'prepare_learn_face',
            prepare_learn_face(),
            transitions={'succeeded':'learn_face','aborted' : 'aborted','preempted':'preempted'})
        # it call the learn state
        smach.StateMachine.add(
            'learn_face',
            learn_face(),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted_info'})
        # it prints the standard error
        smach.StateMachine.add(
            'aborted_info',
            learn_Face_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'learn_face_introspection', sm, '/LFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
