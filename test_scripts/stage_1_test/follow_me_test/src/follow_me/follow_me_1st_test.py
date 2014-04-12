#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Roger Boldu
"""


import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState
from follow_me_1st import follow_me_1st
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


#TODO: it's incomplate, hear we will have to know what ID it will need
class prepare_msg(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],output_keys=['in_learn_person'])

    def execute(self, userdata):
        userdata.in_learn_person='ok'
        rospy.loginfo("preparing msgs")
        return 'succeeded'

class follow_me_1st_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'],
                              output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of follo_me_1st')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'

def main():
    rospy.loginfo('follow_me_1st_test')
    rospy.init_node('follow_me_1st_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    with sm:
        sm.userdata.standard_error='OK'
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'prepare_msg',
            prepare_msg(),
            transitions={'succeeded':'follow_me1st_test','aborted' : 'aborted',
                         'preempted':'preempted'})
        # it call the drop_face state
        smach.StateMachine.add(
            'follow_me1st_test',
            follow_me_1st(),
            transitions={'ELEVATOR':'succeeded', 'LOST':'aborted'})
        smach.StateMachine.add(
            'follow_me_info',
            follow_me_1st_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_me_1st_introspection', sm, '/FM1_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
