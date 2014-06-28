#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLLOW_ME.PY ###
import smach
import rospy
"""
@author: Roger Boldu
"""
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from speech_states.say import text_to_say
from speech_states.listen_and_check_word import ListenWordSM_Concurrent
from follow_learn import LearnPerson
#from speech_states.listen_to import  ListenToSM
#from learn_person import LearnPerson


FOLLOW_GRAMMAR_NAME = 'robocup/followme'

START_FOLLOW_FRASE = "Ok, I'll follow you wherever you want. Stay in front of me while I learn how you look like."
LEARNED_PERSON_FRASE = "OK,  I  am ready to  follow you!"
START_FRASE="Hello, my name is REEM! What do you want me to do today?"






class FollowMeInit(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error','in_learn_person'])

        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standard_error='OK'
            self.userdata.in_learn_person=1
            smach.StateMachine.add('INTRO',
                                   text_to_say(START_FRASE),
                                   transitions={'succeeded': 'Listen','aborted':'INTRO'})

            smach.StateMachine.add('Listen',
                                   ListenWordSM_Concurrent("follow me"),
                                   transitions={'succeeded': 'START_FOLLOWING_COME_CLOSER',
                                                'aborted': 'Listen'})
          
            smach.StateMachine.add('START_FOLLOWING_COME_CLOSER',
                                   text_to_say(START_FOLLOW_FRASE),
                                   transitions={'succeeded': 'SM_LEARN_PERSON','aborted':'aborted'})

            # it learns the person that we have to follow
            smach.StateMachine.add('SM_LEARN_PERSON',
                                   LearnPerson(),
                                   transitions={'succeeded': 'SM_STOP_LEARNING',
                                                'aborted': 'aborted'})

            smach.StateMachine.add('SM_STOP_LEARNING',
                                   text_to_say(LEARNED_PERSON_FRASE),
                                   transitions={'succeeded': 'succeeded','aborted':'aborted'})
