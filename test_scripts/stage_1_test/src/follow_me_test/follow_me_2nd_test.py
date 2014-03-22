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

from follow_me.follow_me_2nd import follow_me_2nd

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


#TODO: it's incomplate, hear we will have to know what ID it will need
class init_2nd_part(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("preparing 2nd part")
        return 'succeeded'

class follow_me_2nd_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of follo_me_1st')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'

def main():
    rospy.init_node('follow_me_2nd_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'init_2nd_part',
            init_2nd_part(),
            transitions={'succeeded':'follow_me2nd_test','aborted' : 'aborted','preempted':'preempted'})
        # it call the drop_face state
        smach.StateMachine.add(
            'follow_me2nd_test',
            follow_me_2nd(),
            transitions={'succeeded':'succeeded','aborted' : 'follow_me_info','preempted':'preempted'})
        smach.StateMachine.add(
            'follow_me_info',
            follow_me_2nd_error(),
            transitions={'succeeded': 'dummy', 'aborted':'dummy'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_me_2nd_introspection', sm, '/FM2_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
