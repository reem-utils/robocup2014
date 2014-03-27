#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Sergi Xavier Ubach Pallàs
"""

import rospy
import smach
import smach_ros
import actionlib
#from smach_ros import SimpleActionState, ServiceState


from listen_to import ListenToSM
"""
    This file tests the listen_to function
    
"""


class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=[])

    def execute(self, userdata):
        rospy.sleep(1) # in seconds

        return 'succeeded'
    
class Speaking_cb(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                input_keys=['asr_userSaid'],
                                output_keys=['standard_error'])
    
    def execute(self, userdata):
        rospy.loginfo("------------------------ '%s'" % userdata.asr_userSaid)
        userdata.standard_error = ''
    
        return 'succeeded'  


def main():
    rospy.init_node('sm_test_listen_to')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        # Using this state to wait and to initialize stuff if necessary (fill up input/output keys for example)
        smach.StateMachine.add(
            'dummy_state',
            DummyStateMachine(),
            transitions={'succeeded': 'ListenToTest'})

        sm.userdata.grammar_name = "JungleParty"
        smach.StateMachine.add('ListenToTest',
            ListenToSM(),
            transitions={'succeeded': 'succe', 'aborted': 'aborted'})
        
        smach.StateMachine.add('succe',
                    Speaking_cb(),
                    transitions={'succeeded': 'succeeded'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'listen_to_test_introspection', sm, '/LISTEN_TO_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
