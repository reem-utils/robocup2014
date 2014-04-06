#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach

from follow_operator import FollowOperator
from check_elevator_sm import look_for_elevator


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


# gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['FOLLOW_OPERATOR'] == 'lost':
        rospy.loginfo(OKGREEN + "It lost the person ends" + ENDC)
        return True
    else:
        rospy.loginfo(OKGREEN + "finish with " + str(outcome_map['FOLLOW_OPERATOR'])  + ENDC)
    # terminate all running states if BAR finished
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "check elevator ends, i have found a elevator" + ENDC)
        return True
        #return 'ELEVATOR'  
    # in all other case, just keep running, don't terminate anything
    return False

def out_cb_follow(outcome_map):
    
    rospy.loginfo(OKGREEN + "maybbe bybe" + ENDC)
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "bye bye elevator" + ENDC)
        return 'ELEVATOR'    
    elif outcome_map['FOLLOW_OPERATOR'] == 'lost':
        return 'LOST'    
    else:
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
    def __init__(self, distToHuman=0.9):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],input_keys=['in_learn_person'])
        
        with self:
            sm=smach.Concurrence(outcomes=['ELEVATOR', 'LOST','preempted'],
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
            
            smach.StateMachine.add('FOLLOW_AND_CHECKING', sm,
                                     transitions={'ELEVATOR':'succeeded',
                                                 'LOST':'aborted','preempted':'succeeded'})
                  
               
            
