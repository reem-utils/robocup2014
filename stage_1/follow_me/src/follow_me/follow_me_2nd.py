#! /usr/bin/env python

import rospy
import smach


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from check_elevator_sm import look_for_elevator_door



class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.sleep(2)
        userdata.standard_error="Dummy"
        return 'succeeded'
     

class dummy_listen(smach.State):
    def __init__(self):
        smach.State.__init__(
                            self,
                            outcomes=['succeeded', 'aborted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        rospy.sleep(2)
        return 'succeeded'
    
class say_out (smach.State):
    
    def __init__ (self):
        smach.State.__init__(
           self,
           outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])
    def execute (self,userdata):
        
        return'succeeded'

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
        return 'succeeded'    
    elif outcome_map['CHECK_DOOR'] == 'succeeded':
        return 'succeeded'    

    return 'aborted'


#I will be hear waiting for the instruction to go out
class follow_me_2nd(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error'])
        
        with self:
            self.userdata.standar_error="ok"
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            
            
            sm=smach.Concurrence(outcomes=['succeeded', 'aborted'],
                                   default_outcome='succeeded',
                                   child_termination_cb = child_term_cb, outcome_cb=out_cb)
    
            
            
            
            with sm:
                
                # it will finisheed with succeeded if it check a door
                
                sm.add('CHECK_DOOR',
                                look_for_elevator_door())
                # here i have to listen if they say me to get out of the lift
                sm.add('LISTEN_OPERATOR_FOR_EXIT',
                                dummy_listen())
                
            smach.StateMachine.add('CONCURRENCE', sm, transitions={'succeeded': 'SAY_OUT',
                                                'aborted': 'aborted'})
            
            
            smach.StateMachine.add('SAY_OUT',
                                   say_out(),
                                   transitions={'succeeded': 'succeeded',
                                    'aborted': 'aborted','preempted':'preempted'})
