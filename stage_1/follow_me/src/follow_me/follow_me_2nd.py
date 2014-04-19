#! /usr/bin/env python

import rospy
import smach


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from check_elevator_sm import look_for_elevator_door
from speech_states.say import text_to_say
from speech_states.say_yes_or_no import SayYesOrNoSM
from speech_states.listen_and_check_word import ListenWordSM, ListenWordSM_Concurrent
            
SAY_OUT_FRASE= "THE DOOR ITS OPEN,  IM GOING OUT"





class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],
            output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"I'M in the second part of the follow_me"+ENDC)
        userdata.standard_error="Dummy"
        return 'succeeded'

        
    # gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['CHECK_DOOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "the door its open" + ENDC)
        return True
    
    if outcome_map['LISTEN_OPERATOR_FOR_EXIT'] == 'succeeded':
        rospy.loginfo(OKGREEN + "the operator say me go out" + ENDC)
        return True

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['LISTEN_OPERATOR_FOR_EXIT'] == 'succeeded':
        return 'OPERATOR'    
    elif outcome_map['CHECK_DOOR'] == 'succeeded':
        return 'DOOR_OPEN'    
    return 'aborted'


#I will be hear waiting for the instruction to go out
class follow_me_2nd(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standar_error="ok"
            self.userdata.word_to_listen=None
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'CONCURRENCE',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            sm=smach.Concurrence(outcomes=['DOOR_OPEN', 'OPERATOR','aborted'],
                                   default_outcome='DOOR_OPEN',input_keys=['word_to_listen'],
                                   child_termination_cb = child_term_cb, outcome_cb=out_cb)
    

            with sm:
                
                # it will finisheed with succeeded if it check a door
                
                sm.add('CHECK_DOOR',
                                look_for_elevator_door())
                # here i have to listen if they say me to get out of the lift
                sm.add('LISTEN_OPERATOR_FOR_EXIT',
                                ListenWordSM_Concurrent("go out"))
                
            smach.StateMachine.add('CONCURRENCE', sm, transitions={'OPERATOR': 'SAY_OUT',
                                                                   'DOOR_OPEN':'SAY_OUT',
                                                                   'aborted':'CONCURRENCE'})
            
            # it says i'm going out
            smach.StateMachine.add('SAY_OUT',
                                   text_to_say(SAY_OUT_FRASE),
                                   transitions={'succeeded': 'succeeded',
                                    'aborted': 'aborted','preempted':'preempted'})


            
            
            
            
            #TODO: i Have a book!!
            
            
