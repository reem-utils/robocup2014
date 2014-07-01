#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon 31 June 3:40:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""

import rospy
import smach
import actionlib
from rospy.core import rospyinfo
from smach_ros import ServiceState

from navigation_states.nav_to_poi import nav_to_poi
from speech_states.say import text_to_say 

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class prepare_state(smach.State):
    
    def __init__(self,nav_to_poi_name,tts_text):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['tts_text','nav_to_poi_name'],
                             output_keys=['tts_text','nav_to_poi_name'])
    
        self.nav_to_poi_name = nav_to_poi_name
        self.tts_text = tts_text
        
    def execute(self, userdata):
        
        if userdata.tts_text == None :
            userdata.tts_text = self.tts_text
        if userdata.nav_to_poi_name == None :
            userdata.nav_to_poi_name = self.nav_to_poi_name
        
        return 'succeeded'


class nav_to_poi_and_say(smach.StateMachine): 
    """
    This state will move to the POI specified and at the same time says a defined phrase.
    
    - Input Keys:
        @key nav_to_poi_name
        @key tts_text

    - Output keys:
        @key standard_error: inform what is the problem
         
    - No io_keys.

    """
    def __init__(self, nav_to_poi_name = None, tts_text = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['tts_text','nav_to_poi_name'], 
                                 output_keys=['standard_error'])
        with self:
            self.userdata.tts_lang='en_US'
            self.userdata.tts_wait_before_speaking=0
            
            self.userdata.standard_error='OK'
            
            smach.StateMachine.add(
                                'INIT_VAR',
                                prepare_state(nav_to_poi_name, tts_text),
                                transitions={'succeeded': 'Concurrence_Say_Nav', 'aborted': 'aborted', 
                                'preempted': 'preempted'})    
            
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'preempted','aborted'],
                                        default_outcome='succeeded',
                                        input_keys=['tts_text',
                                                   'nav_to_poi_name',
                                                   'tts_wait_before_speaking',
                                                   'tts_lang'])
            with sm_conc:
                sm_conc.add('Say_conc',
                                text_to_say())

                sm_conc.add('Nav_to_poi_conc',
                                nav_to_poi())
                
            smach.StateMachine.add('Concurrence_Say_Nav', 
                                   sm_conc,
                                   transitions={'succeeded':'succeeded',
                                                 'aborted':'aborted',
                                                 'preempted':'preempted'})
            
def main():
    rospy.loginfo('nav_to_poi_and_say')
    rospy.init_node('nav_to_poi_and_say_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:      
        sm.userdata.tts_text = 'I am going to place'
        sm.userdata.nav_to_poi_name = 'kitchen'
        
        smach.StateMachine.add(
            'nav_to_poi_and_say',
            nav_to_poi_and_say(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()
