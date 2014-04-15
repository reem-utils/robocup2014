#!/usr/bin/env python

"""
Author:  Sergi Xavier Ubach
Email: sxubach@gmail.com

22 Feb 2014
"""

import rospy
import smach

from smach_ros import SimpleActionState
#from text_to_speech.msg import SoundAction, SoundGoal 
from pal_interaction_msgs.msg import SoundAction, SoundGoal 

# Constants
TTS_TOPIC_NAME = '/sound'

class prepareData(smach.State):
    
    def __init__(self, text, wait_time, lang):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['tts_text','tts_wait_before_speaking','tts_lang'], output_keys=['tts_text','tts_wait_before_speaking','tts_lang'])
        self.text = text
        self.wait_time = wait_time
        self.lang = lang
        
    def execute(self, userdata):
           
        if not self.text and not userdata.tts_text:
            rospy.logerr("Text isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.tts_text = self.text if self.text else userdata.tts_text   
        #Priority in userdata
        userdata.tts_wait_before_speaking = userdata.tts_wait_before_speaking if userdata.tts_wait_before_speaking else self.wait_time 
        userdata.tts_lang = userdata.tts_lang if userdata.tts_lang else self.lang

        return 'succeeded'
    
class createSayGoal(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['tts_text','tts_wait_before_speaking','tts_lang'], output_keys=['tts_speech_goal'])
    
    # Create a SoundGoal with text, lang_id and wait_before_speaking. 
    def create_say_goal(self, tts_text, tts_wait_before_speaking, tts_lang):
        rospy.loginfo(tts_text)
        say_goal = SoundGoal()
        say_goal.wait_before_speaking.secs = tts_wait_before_speaking 
        say_goal.wait_before_speaking.nsecs = 0
        say_goal.text = tts_text
        say_goal.lang_id = tts_lang
         
        return say_goal
    
    def execute(self, userdata):
                
        say_goal = self.create_say_goal(userdata.tts_text, 
                                    userdata.tts_wait_before_speaking,
                                    userdata.tts_lang)

        userdata.tts_speech_goal = say_goal
        return 'succeeded'
 

class text_to_say(smach.StateMachine):
    """
        To use say you need to indicate the text to be said. By default it waits 0 seconds before
        speaking and uses en_US language and don't use the nsecs wait option.
        
        smach.StateMachine.add(
        	    'SaySM',
        	    text_to_say("I'm working"),
        	    transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    """

                    
    def __init__(self, text=None, text_cb=None, wait_before_speaking=0, lang='en_US'):
        """
           Constructor for text_to_say.

            @param text: the text to say
            @param text_cb: a callback returning the text to speak
            @param wait_before_speaking: how long to wait before speaking
            @param lang: The language that use to speak  
        """

        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['tts_text', 'tts_wait_before_speaking', 'tts_lang'],
                                 output_keys=[])
        
        with self: 
            
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            
            smach.StateMachine.add('PrepareData',
                                   prepareData(text, wait_before_speaking, lang),
                                   transitions={'succeeded':'CreateSayGoal', 'aborted':'aborted'})
            
            smach.StateMachine.add('CreateSayGoal',
                                   createSayGoal(),
                                   transitions={'succeeded':'RobotSay', 'aborted':'aborted'})        
            
            smach.StateMachine.add('RobotSay', 
                                SimpleActionState(TTS_TOPIC_NAME,
                                                   SoundAction,
                                                   goal_key='tts_speech_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error']), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})

