#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 30 17:00:00 2013

@author: Roger Boldu
@email: roger.boldu@gmail.com
"""

import rospy
import smach
import actionlib
from rospy.core import rospyinfo
from smach_ros import ServiceState

from manipulation_states.play_motion_sm import play_motion_sm
from speech_states.say import text_to_say 

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class init_var(smach.State):
    
    def __init__(self,type_movement,tts_text):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['tts_text','type_movment'],
                             output_keys=['tts_text','type_movment','manip_motion_to_play'],)
    
        self.type_movement=type_movement
        self.tts_text=tts_text
        
    def execute(self, userdata):
        
        
        if   userdata.tts_text == None :
            userdata.tts_text=self.tts_text
        if  userdata.type_movment == None :
            userdata.type_movment=self.type_movement
        
        userdata.manip_motion_to_play=userdata.type_movment
        return 'succeeded'


class acknowledgment(smach.StateMachine): 
    """
    This state will move the head and say information.
    @Inputs userdata.tts_text // tts_text : it's the text that will say
            userdata.type_movment //  type_movment : it can be yes or not
     for defauld is yes, you only have to put the sentence
    output keys:
        standard_error: inform what is the problem
        succeeded : if it was possible
        aborted: if it's not possible
        preempted if some one have say 
    No io_keys.

    @example :acknowledgment(type_movement="yes", tts_text="OK, i'm ready, do you wan't to be my friend")
    """
    def __init__(self,type_movement="yes",tts_text=''):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=[], 
                                 output_keys=['standard_error'])
        
        self.type_movement=type_movement
        self.tts_text=tts_text
        
        with self:
            self.userdata.tts_text=None
            self.userdata.type_movment = None
            self.userdata.tts_lang=None
            self.userdata.tts_wait_before_speaking=0
            self.userdata.standard_error='OK'
            self.userdata.manip_time_to_play=30
            
            smach.StateMachine.add(
                                'INIT_VAR',
                                init_var(self.type_movement,
                                         self.tts_text),
                                transitions={'succeeded': 'PUT_MOVMENT', 'aborted': 'aborted', 
                                'preempted': 'preempted'})    
            
         

            sm=smach.Concurrence(outcomes=['succeeded', 'preempted','aborted'],
                                   default_outcome='succeeded',input_keys=['tts_text',
                                                                           'manip_motion_to_play',
                                                                           'manip_time_to_play',
                                                                           'tts_wait_before_speaking',
                                                                           'tts_lang'])
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None

            with sm:
                

                sm.add('SAY',
                                text_to_say())

                sm.add('MOVE',
                                play_motion_sm())
                
            smach.StateMachine.add('PUT_MOVMENT', sm,
                                     transitions={'succeeded':'succeeded',
                                                 'aborted':'aborted','preempted':'preempted'})
            

