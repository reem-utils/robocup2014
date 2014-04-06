#! /usr/bin/env python

import rospy
import smach


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from check_elevator_sm import look_for_elevator_door
from speech_states.say import text_to_say
            
SAY_OUT_FRASE= "OK IM GOING OUT"
COMPROBATE_GO_OUT="DO YOU WANT TO GO OUT?"


class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"I'M in the second part of the follow_me"+ENDC)
        rospy.sleep(2)
        userdata.standard_error="Dummy"
        return 'succeeded'
     

class dummy_listen(smach.State):
    def __init__(self):
        smach.State.__init__(
                            self,
                            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        while (1):
            if self.preempt_requested():
                return 'preempted'
            
        return 'succeeded'
    
class say_out (smach.State):
    
    def __init__ (self):
        smach.State.__init__(
           self,
           outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])
    def execute (self,userdata):
        
        return'succeeded'


class listen_yes_or_not_dummy (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
    def execute(self):
        rospy.loginfo("i'm in¡ the dumy state listen if its yeas or not")
        return 'succeeded'
        
    # gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['CHECK_DOOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "the door it's open" + ENDC)
        return True
    
    if outcome_map['LISTEN_OPERATOR_FOR_EXIT'] == 'succeeded':
        rospy.loginfo(OKGREEN + "the operator say me go out" + ENDC)
        return True

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['LISTEN_OPERATOR_FOR_EXIT'] == 'succeeded':
        return 'DOOR_OPEN'    
    elif outcome_map['CHECK_DOOR'] == 'succeeded':
        return 'OPERATOR'    

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
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'CONCURRENCE',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            
            
            sm=smach.Concurrence(outcomes=['DOOR_OPEN', 'OPERATOR'],
                                   default_outcome='DOOR_OPEN',
                                   child_termination_cb = child_term_cb, outcome_cb=out_cb)
    
            
            
            
            with sm:
                
                # it will finisheed with succeeded if it check a door
                
                sm.add('CHECK_DOOR',
                                look_for_elevator_door())
                # here i have to listen if they say me to get out of the lift
                sm.add('LISTEN_OPERATOR_FOR_EXIT',
                                dummy_listen())
                
            smach.StateMachine.add('CONCURRENCE', sm, transitions={'OPERATOR': 'SAY_OUT',
                                                                   'DOOR_OPEN':'SAY_DO_YOU_WANT'})
            
            
            # it says i'm going out
            smach.StateMachine.add('SAY_OUT',
                                   text_to_say(COMPROBATE_GO_OUT),
                                   transitions={'succeeded': 'succeeded',
                                    'aborted': 'aborted','preempted':'preempted'})
            # i detect that the door it's open, and i will asck to the operator if it want to go out
            smach.StateMachine.add('SAY_DO_YOU_WANT',
                                   text_to_say(COMPROBATE_GO_OUT),
                                   transitions={'succeeded': 'YES_OR_NOT',
                                    'aborted': 'aborted','preempted':'preempted'})
            #comprobate that the order it's ok
            smach.StateMachine.add('YES_OR_NOT',
                                   listen_yes_or_not_dummy(),
                                   transitions={'succeeded': 'succeeded',
                                    'aborted': 'INIT_VAR','preempted':'preempted'})

            
            
            
            
            
            
            