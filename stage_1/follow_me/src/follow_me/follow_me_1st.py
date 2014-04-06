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

    # terminate all running states if BAR finished
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Find_faces ends" + ENDC)
        return True

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['CHECK_ELEVATOR'] == 'succeeded':
        return 'ELEVATOR'    
    elif outcome_map['FOLLOW_OPERATOR'] == 'lost':
        return 'LOST'    
    else:
        return 'LOST' # i don't like this return never it have to be
    
    
# TODO: now i'm not listen hear... maybe will be intesting
class follow_me_1st(smach.Concurrence):
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

        smach.Concurrence.__init__(self,outcomes=['ELEVATOR', 'LOST'],
                                   default_outcome='LOST',input_keys=["in_learn_person"],
                                   child_termination_cb = child_term_cb,
                                   outcome_cb=out_cb)
        
       # rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", distToHuman)
    
        with self:  
    
            self.userdata.standard_error='OK'
            smach.Concurrence.add('FOLLOW_OPERATOR',
                            FollowOperator())
            #This it will return if it's in the elevator, and if it's in the elevator
            # it have to say: i'm in the elevator
            # it have to sent a goal in a less distance of the operator
            smach.Concurrence.add('CHECK_ELEVATOR',
                            look_for_elevator())
                  
               
            
