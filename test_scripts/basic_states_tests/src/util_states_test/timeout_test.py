#! /usr/bin/env python

"""
Created on 15/03/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach

from util_states.timeout import TimeOut
from util_states.sleeper import Sleeper
    
def child_term_cb(outcome_map):
    
    #If time passed, we terminate all running states
    if outcome_map['TimeOut'] == 'succeeded':
        rospy.loginfo('TimeOut finished')
        return True
    if outcome_map['Sleep'] == 'succeeded':
        rospy.loginfo('Sleeper Finished')
        return True
    #By default, just keep running
    return False
    
def out_cb(outcome_map):
    if outcome_map['TimeOut'] == 'succeeded':
        return 'succeeded'   
    elif outcome_map['Sleep'] == 'succeeded':
        return 'succeeded'
    else:
        return 'aborted'
    

def main():
    """
        Test the timeout and the sleeper
    """
    
    rospy.init_node('basic_functionalities')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
        sm.userdata.wait_time = None
        sm.userdata.sleep_time = None
        sm.userdata.standard_error = ''
        
        # Concurrence
        sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                    default_outcome='succeeded',
                                    input_keys=['wait_time', 'sleep_time'],
                                    output_keys=['standard_error'],
                                    child_termination_cb = child_term_cb,
                                    outcome_cb=out_cb)
        with sm_conc: 
            smach.Concurrence.add('Sleep', Sleeper(20))
            
            smach.Concurrence.add('TimeOut', TimeOut(5))
        
        smach.StateMachine.add(
            'Concurrence',
            sm_conc,
            transitions={'succeeded': 'succeeded', 'aborted': 'Concurrence'}) 
   
       

    sm.execute()



if __name__ == '__main__':
    main()