#!/usr/bin/env python

"""
Author:  Roger Boldu
Email: roger.boldu@gmail.com

"""

import rospy
import smach

from say import text_to_say


class prepareData(smach.State):
    
    def __init__(self,enable=True):
    
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                              input_keys=['tts_text','tts_wait_before_speaking','tts_lang'], output_keys=['tts_text','tts_wait_before_speaking','tts_lang'])
        self.enable=enable

    def execute(self, userdata):
           
        if self.enable :
            return 'succeeded'
        else :
            return 'aborted'
    
 

class say_with_enable(smach.StateMachine):
    """
        To use say you need to indicate the text to be said. By default it waits 0 seconds before
        speaking and uses en_US language and don't use the nsecs wait option.
        
        smach.StateMachine.add(
                'SaySM',
                text_to_say("I'm working"),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    """

                    
    def __init__(self, text=None, text_cb=None, wait_before_speaking=0, lang='en_US',enable=True,wait=True):
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
        
        self.text=text
        self.enable=enable
        self.wait=wait
        with self: 
            
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            
            smach.StateMachine.add('PrepareData',
                                   prepareData(self.enable),
                                   transitions={'succeeded':'CreateSayGoal', 'aborted':'succeeded', 'preempted':'preempted'})
            
            smach.StateMachine.add('CreateSayGoal',
                                   text_to_say(self.text,wait=self.wait),
                                   transitions={'succeeded':'succeeded', 'aborted':'aborted','preempted':'preempted'})        

