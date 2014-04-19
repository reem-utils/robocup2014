#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach

from follow_operator import FollowOperator
from check_elevator_sm import look_for_elevator
from speech_states.listen_and_check_word import ListenWordSM_Concurrent
from speech_states.say import text_to_say


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

SAY_DOR_NOT_CLOSE="That was extrange, i din't find de lift"


# gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['FOLLOW_OPERATOR'] == 'lost':
        rospy.loginfo(OKGREEN + "It lost the person ends" + ENDC)
        return True

    # terminate all running states if BAR finished
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "check elevator ends, i have found a elevator" + ENDC)
        return True
    
        # terminate all running states if BAR finished
    if outcome_map['LISTEN_OPERATOR_FOR_EXIT_SENTENCE'] == 'succeeded':
        rospy.loginfo(OKGREEN + "operator say go out..." + ENDC)
        return True
        #return 'ELEVATOR'  
    # in all other case, just keep running, don't terminate anything
    return False

def out_cb_follow(outcome_map):
    
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        return 'ELEVATOR'    
    elif outcome_map['FOLLOW_OPERATOR'] == 'lost':
        return 'LOST'    
    elif outcome_map['LISTEN_OPERATOR_FOR_EXIT_SENTENCE']=='succeeded':
        return 'OPERATOR'
    else :    
        return 'LOST' # i don't like this return never it have to be
    
    
# TODO: now i'm not listen hear... maybe will be intesting
class follow_me_1st(smach.StateMachine):
    """
    Executes a SM that do the first part of the follow_me.
    This part consist in follow the operator,
    2 person it will cross, one of them it will stops for 3 seconds 
    this part of the follow_me it will be finished when the robot 
    arrives to the "small room" or the robot i comes lost
        
    It creates a concurrence state machine for:
        Listen
        Follow
        CHECK_ELEVATOR
    """
    def __init__(self, distToHuman=0.5):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted','operator_say_out'],
                                    input_keys=['in_learn_person'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standar_error="ok"
            sm=smach.Concurrence(outcomes=['ELEVATOR', 'LOST','preempted','OPERATOR'],
                                    default_outcome='ELEVATOR',input_keys=["in_learn_person"],
                                    child_termination_cb = child_term_cb,
                                    outcome_cb=out_cb_follow,output_keys=[])
            
            with sm:
    
                smach.Concurrence.add('FOLLOW_OPERATOR',
                                FollowOperator(distToHuman))
                #This it will return if it's in the elevator, and if it's in the elevator
                # it have to say: i'm in the elevator
                # it have to sent a goal in a less distance of the operator
                smach.Concurrence.add('CHECK_ELEVATOR',
                                look_for_elevator())
                                # here i have to listen if they say me to get out of the lift
                sm.add('LISTEN_OPERATOR_FOR_EXIT_SENTENCE',
                                ListenWordSM_Concurrent("exit"))
                
            
            smach.StateMachine.add('FOLLOW_AND_CHECKING', sm,
                                     transitions={'ELEVATOR':'succeeded',
                                                 'LOST':'aborted','OPERATOR':'SAY_EXTRANGE','preempted':'succeeded'})
            
                    # Read from server
            smach.StateMachine.add('SAY_EXTRANGE',text_to_say(SAY_DOR_NOT_CLOSE),
                                   transitions={'succeeded': 'operator_say_out', 'aborted': 'operator_say_out', 'preempted': 'preempted'})
            
            
                  
               
            
