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
from pal_interaction_msgs.msg import SoundActionGoal 

# Constants
TTS_TOPIC_NAME = '/sound'

class prepareData(smach.State):
    
    def __init__(self, text, wait_time, lang,wait):
        
        smach.State.__init__(self, outcomes=['no_wait','aborted', 'preempted','wait'], 
                            input_keys=['tts_text','tts_wait_before_speaking','tts_lang'],
                            output_keys=['tts_text','tts_wait_before_speaking','tts_lang'])
        self.text = text
        self.wait_time = wait_time
        self.lang = lang
        self.wait=wait
        
    def execute(self, userdata):
           
        if not self.text and not userdata.tts_text:
            rospy.logerr("Text isn't set")
            return 'aborted'
        if self.preempt_requested():
            return 'preempted'
        #Priority in init
        userdata.tts_text = self.text if self.text else userdata.tts_text   
        #Priority in userdata
        userdata.tts_wait_before_speaking = userdata.tts_wait_before_speaking if userdata.tts_wait_before_speaking else self.wait_time 
        userdata.tts_lang = userdata.tts_lang if userdata.tts_lang else self.lang
        if self.wait :
            return 'wait'
        else :
            return 'no_wait'


class prepareData_old(smach.State):
    
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
        if self.preempt_requested():
            return 'preempted'
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
        if self.preempt_requested():
            return 'preempted'
                
        say_goal = self.create_say_goal(userdata.tts_text, 
                                    userdata.tts_wait_before_speaking,
                                    userdata.tts_lang)

        userdata.tts_speech_goal = say_goal
        return 'succeeded'
    
class createSayGoal_noWait(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['tts_text','tts_wait_before_speaking','tts_lang'], output_keys=['tts_speech_goal'])
    
    # Create a SoundGoal with text, lang_id and wait_before_speaking. 
    def create_say_goal(self, tts_text, tts_wait_before_speaking, tts_lang):
        
        rospy.loginfo(tts_text)
        say_goal = SoundActionGoal()
        say_goal.goal.text=tts_text
        say_goal.goal.lang_id = tts_lang  
        return say_goal
    
    def execute(self, userdata):
        if self.preempt_requested():
            return 'preempted'
                
        say_goal = self.create_say_goal(userdata.tts_text, 
                                    userdata.tts_wait_before_speaking,
                                    userdata.tts_lang)

        userdata.tts_speech_goal = say_goal
        return 'succeeded'
 
 

class send_goal(smach.State):
    def __init__(self,say_pub):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['tts_speech_goal'], output_keys=[])
        self.say_pub=say_pub
    def execute(self, userdata):
        
        self.say_pub.publish(userdata.tts_speech_goal)
        
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

                    
    def __init__(self, text=None, text_cb=None, wait_before_speaking=0, lang='en_US',wait=True):
        """
           Constructor for text_to_say.

            @param text: the text to say
            @param text_cb: a callback returning the text to speak
            @param wait_before_speaking: how long to wait before speaking
            @param lang: The language that use to speak  
            @param wait : it means that will wait or not
        """

        #Initialization of the SMACH State machine
        self.say_pub= rospy.Publisher('/sound/goal', SoundActionGoal)
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['tts_text', 'tts_wait_before_speaking', 'tts_lang'],
                                 output_keys=[])
        
        with self: 
            
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            
            smach.StateMachine.add('PrepareData',
                                   prepareData(text, wait_before_speaking, lang),
                                   transitions={'wait':'CreateSayGoal', 'aborted':'aborted', 'preempted':'preempted','no_wait':'create_goal_no_wait'})
            
            smach.StateMachine.add('CreateSayGoal',
                                   createSayGoal(),
                                   transitions={'succeeded':'RobotSay', 'aborted':'aborted','preempted':'preempted'})        
            
            smach.StateMachine.add('RobotSay', 
                                SimpleActionState(TTS_TOPIC_NAME,
                                                   SoundAction,
                                                   goal_key='tts_speech_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error']), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            
            
            
            smach.StateMachine.add('create_goal_no_wait',
                                   createSayGoal_noWait(),
                                   transitions={'succeeded':'SEND_GOAL', 'aborted':'aborted'})
            
            smach.StateMachine.add('SEND_GOAL',
                       send_goal(self.say_pub),
                       transitions={'succeeded':'succeeded', 'aborted':'aborted','preempted':'preempted'})
                        
            
            

class text_to_say_old(smach.StateMachine):
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
                                   transitions={'succeeded':'CreateSayGoal', 'aborted':'aborted', 'preempted':'preempted'})
            
            smach.StateMachine.add('CreateSayGoal',
                                   createSayGoal(),
                                   transitions={'succeeded':'RobotSay', 'aborted':'aborted','preempted':'preempted'})        
            
            smach.StateMachine.add('RobotSay', 
                                SimpleActionState(TTS_TOPIC_NAME,
                                                   SoundAction,
                                                   goal_key='tts_speech_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error']), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})
