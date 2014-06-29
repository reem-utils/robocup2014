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
from follow_me.follow_learn import LearnPerson
#from speech_states.listen_to import  ListenToSM
#from learn_person import LearnPerson



START_FRASE="Hello, my name is REEM! I am here to learn about this restaurant  , i need few seconds to prepare,  please stay near of my eyes"
LETS_GO="OK, i am ready to start learning"




class restaurantInit(smach.StateMachine):
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
                                   transitions={'succeeded': 'Learn','aborted':'aborted'})

            # it learns the person that we have to follow
            smach.StateMachine.add('Learn',
                                   LearnPerson(),
                                   transitions={'succeeded': 'SM_STOP_LEARNING',
                                                'aborted': 'aborted'})

            smach.StateMachine.add('SM_STOP_LEARNING',
                                   text_to_say(LETS_GO),
                                   transitions={'succeeded': 'succeeded','aborted':'aborted'})
