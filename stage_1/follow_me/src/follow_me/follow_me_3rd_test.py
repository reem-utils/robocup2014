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
from follow_me_3rd import follow_me_3rd
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


#TODO: it's incomplate, hear we will have to know what ID it will need
class prepare_msg(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("preparing msgs")
        return 'succeeded'

class follow_me_3rd_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of follo_me_3rd')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'

def main():
    rospy.init_node('follow_me_2nd_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    with sm:
        sm.userdata.standard_error='OK'
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'prepare_msg',
            prepare_msg(),
            transitions={'succeeded':'follow_me3rd_test','aborted' : 'aborted','preempted':'preempted'})
        # it call the drop_face state
        smach.StateMachine.add(
            'follow_me3rd_test',
            follow_me_3rd(),
            transitions={'succeeded':'succeeded','aborted' : 'follow_me_info','preempted':'preempted'})
        smach.StateMachine.add(
            'follow_me_info',
            follow_me_3rd_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_me_3rd_introspection', sm, '/FM3_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
