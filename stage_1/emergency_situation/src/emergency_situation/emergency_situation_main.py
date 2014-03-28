#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 15 12:00:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
import smach_ros
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from emergency_situation.emergency_situation_sm import emergency_situation_sm
#from emergency_situation.Get_Person_Desired_Object import Get_Person_Desired_Object
#from emergency_situation.Save_People_Emergency import Save_People_Emergency

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


def main():
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

        with sm:           
            smach.StateMachine.add(
                'Dummy',
                DummyStateMachine(),
                transitions={'succeeded':'Emergency_situation', 'aborted':'Emergency_situation', 'preempted':'Emergency_situation'})
            
            sm.userdata.tts_wait_before_speaking = 0
            smach.StateMachine.add(
                'Emergency_situation',
                emergency_situation_sm(),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})


        sis = smach_ros.IntrospectionServer(
            'emergency_situation_introspection', sm, '/SM_ROOT')
        sis.start()
    
        sm.execute()
    
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('Emergency_situation_MAIN')
    main()