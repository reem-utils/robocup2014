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
from follow_me_init import FollowMeInit
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



class follow_me_init_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of follow_me_init')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'

def main():
    rospy.init_node('follow_me_init_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # it call the drop_face state
        sm.userdata.standard_error='ok'
        #sm.userdata.standard_error=33
        smach.StateMachine.add(
            'follow_init_test',
            FollowMeInit(),
            transitions={'succeeded':'succeeded','aborted' : 'follow_me_info','preempted':'preempted'})
        
        smach.StateMachine.add(
            'follow_me_info',
            follow_me_init_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_me_init_introspection', sm, '/FMI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
