#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 22/03/2014

@author: Cristina De Saint Germain
'''

import rospy
import smach
import smach_ros
import actionlib

from cocktail_party_sm import CocktailPartySM
from speech_states.say import text_to_say
from navigation_states.nav_to_poi import nav_to_poi
from util_states.concurrence_with_time import ConcurrenceTime

def main():
    rospy.init_node('cocktail_party')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
        STATES = [CocktailPartySM()]
        STATE_NAMES = ["CocktailPartySM"]
    
        # We have 10 minuts -> 600 sec
#         smach.StateMachine.add(
#             "Cocktail_test",
#             ConcurrenceTime(states=STATES, state_names=STATE_NAMES, timeout=570),
#             transitions={'succeeded': 'leaving_arena', 'aborted': "Say_timeout"})
        smach.StateMachine.add(
            "Cocktail_test",
            CocktailPartySM(),
            transitions={'succeeded': 'leaving_arena', 'aborted': "Say_timeout"})
            
            
        # Say TimeOut
        smach.StateMachine.add(
            'Say_timeout',
            text_to_say("My time to do the test is over. I going to leave the arena", wait=False),
            transitions={'succeeded': 'leaving_arena', 'aborted': 'leaving_arena'})

        # Leaving the arena  
        smach.StateMachine.add(
            'leaving_arena',
            nav_to_poi('leave_arena'),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'}) 
    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'cocktail_party', sm, '/CP_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
