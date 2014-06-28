#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Roger Boldu
"""


import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState

from speech_states.say import text_to_say
from follow_me.follow_operator import FollowOperator
from follow_me.follow_learn import LearnPerson
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

class follow_operator_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'],
                              output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of follo_me_1st')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'

def main():
    rospy.loginfo('follow_operator_TEST')
    rospy.init_node('follow_operator_TEST')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    with sm:
        sm.userdata.standard_error='OK'
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'prepare_msg',
            prepare_msg(),
            transitions={'succeeded':'learn_operator','aborted' : 'aborted','preempted':'preempted'})
        smach.StateMachine.add(
            'learn_operator',
            LearnPerson(),
            transitions={'succeeded':'follow_operator','aborted' : 'aborted','preempted':'preempted'})
        # it call the drop_face state
        smach.StateMachine.add(
            'follow_operator',
            FollowOperator(),
            transitions={'lost':'say_lost','succeeded' : 'follow_me_info'})

        

        sm.userdata.tts_text="i have lose the person"
        sm.userdata.tts_wait_before_speaking=0
        smach.StateMachine.add('say_lost',
                               text_to_say(),
                               transitions={'succeeded': 'follow_me_info','aborted':'follow_me_info'})       
        
    
        smach.StateMachine.add(
                               'follow_me_info',
                               follow_operator_error(),
                               transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'follow_operator_test', sm, '/FO_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
