#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLLOW_ME.PY ###
import roslib
roslib.load_manifest('follow_me')
import smach
import rospy

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from speech_states.say import text_to_say
from speech_states.listen_to import  ListenToSM
 
FOLLOW_GRAMMAR_NAME = 'robocup/followme'

START_FOLLOW_FRASE = "Ok, I'll follow you wherever you want. Please come a bit closer if you are too far, then Please stay still while I learn how you are."
LEARNED_PERSON_FRASE = "Let's go buttercup."

class wait_for_stard(smach.StateMachine):

    def __init__(self): 
        smach.State.__init__(self, input_keys=['asr_userSaid'],
                             outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        
        
        if userdata.asr_userSaid=="Follow me":
            return 'succeded' # todo locks to .gram
        else :
            print (FAIL +"I listen diferent a diferent thing   " +  str(userdata.asr_userSaid) + ENDC)
            return 'aborted'
        return 'succeeded'

#Main
class FollowMe(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:

            self.userdata.tts_text="Hello, my name is REEM! What do you want me to do today?"
            self.userdata.tts_wait_before_speaking=0
            smach.StateMachine.add('INTRO',
                                   text_to_say(),
                                   transitions={'succeeded': 'FOLLOW_ME_COMMAND'})

            ### 2. Follow me command
            # todo create a State waiting if they say that i'm looking for
            self.userdata.grammar_name="follow_me.gram" #TODO ha de ser igual que la del sergi
            smach.StateMachine.add('Listen',
                                   ListenToSM(),
                                   transitions={'succeeded': 'FOLLOW_ME_COMMAND',
                                                'aborted': 'Listen'})
          
            # it locks if it's the command correct
            smach.StateMachine.add('FOLLOW_ME_COMMAND',
                                   wait_for_stard(),
                                   transitions={'succeeded': 'START_FOLLOWING_COME_CLOSER',
                                                'aborted': 'Listen'})

            start_text = START_FOLLOW_FRASE
            self.userdata.tts_text="Ok, I'll follow you wherever you want.
             Please come a bit closer if you aretoo far, then Please stay still while I learn how you are."
            smach.StateMachine.add('START_FOLLOWING_COME_CLOSER',
                                   SpeakActionState(start_text),
                                   transitions={'succeeded': 'SM_LEARN_PERSON'})

            smach.StateMachine.add('SM_LEARN_PERSON',
                                   LearnPerson(),
                                   transitions={'succeeded': 'FOLLOW_ME',
                                                'aborted': 'aborted'},
                                   remapping={'PT_Id_of_person': 'out_targetId',
                                              'LP_all_person_data': 'out_personTrackingData'})

            learned_text = LEARNED_PERSON_FRASE
            smach.StateMachine.add('START_FOLLOWING_COME_CLOSER',
                                   SpeakActionState(learned_text),
                                   transitions={'succeeded': 'SM_LEARN_PERSON'})

            smach.StateMachine.add('FOLLOW_ME',
                                   FollowAndStop(),
                                   remapping={'in_targetId': 'out_targetId',
                                              'in_personTrackingData': 'out_personTrackingData'},
                                   transitions={'succeeded': 'succeeded'})