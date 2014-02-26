#!/usr/bin/env python

"""
Author:  Sergi Ubach
Email: sxubach@dsfjakldfa
22 Feb 2014
"""
# Basat en tts, play_sound_sm, sound_action de l'any pasat i nav_to_coord d'aquest

import rospy
import smach
from smach_ros import SimpleActionState

from text_to_speech.msg import SoundAction, SoundGoal #NO SE SI EXISTEIX 

# Constants
TTS_TOPIC_NAME = '/sound'

"""
To use say you need the following variables, tts_text for the text meant to be said and tts_wait_before_speaking
to indicate how many seconds you want to wait.

sm.userdata.tts_text = "I'm working"
sm.userdata.tts_wait_before_speaking = 0

smach.StateMachine.add(
            'SaySM',
            text_to_say(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
            
By default "say" uses en_US lenguage and don't use the nsecs wait option.
If needed change the code underneath

"""


class createSayGoal(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['tts_text','tts_wait_before_speaking'], output_keys=['tts_speech_goal'])
        
    def create_say_goal(self, tts_text, tts_wait_before_speaking):
        """Create a SoundGoal with text,lang_id and wait_befor speaking(in secs & nsecs)
        Returns a SoundGoal"""
        say_goal = SoundGoal()
        say_goal.wait_before_speaking.secs = tts_wait_before_speaking 
        say_goal.wait_before_speaking.nsecs = 0
        say_goal.text = tts_text
        say_goal.lang_id = 'en_US'
         
        return say_goal
    
    def execute(self, userdata):
        say_goal = self.create_say_goal(userdata.tts_text, 
                                    userdata.tts_wait_before_speaking)

        userdata.tts_speech_goal = say_goal
        return 'succeeded'
 
   

class text_to_say(smach.StateMachine):

    def __init__(self):
        """
        Constructor for nav_to_coord.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['tts_text', 'tts_wait_before_speaking'],
                                 output_keys=['standard_error'])

        with self: 

            smach.StateMachine.add('CreateSayGoal',
                                   createSayGoal(),
                                   transitions={'succeeded':'RobotSay', 'aborted':'aborted'})
            
            def say_res_cb(userdata, result_status, result):

                if result_status != 3: # 3 == SUCCEEDED
                    if result_status == 4: 
                        userdata.standard_error = "Aborted navigation goal (maybe we didn't get there?)"
                        print userdata.standard_error
                    elif result_status == 5: # We havent got a rejected yet, maybe never happens
                        userdata.standard_error = "Rejected navigation goal (maybe the goal is outside of the map or in a obstacle?)"
                        print userdata.standard_error
                    return 'aborted'
                else:
                    userdata.standard_error = ""
                    return 'succeeded'
        
            
            smach.StateMachine.add('RobotSay', 
                                SimpleActionState(TTS_TOPIC_NAME,
                                                   SoundAction,
                                                   goal_key='tts_speech_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=say_res_cb), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})

