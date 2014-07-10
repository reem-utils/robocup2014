#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
10 July 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
from util_states.timeout import TimeOut
from util_states.sleeper import Sleeper
from speech_states.say import text_to_say
from navigation_states.nav_to_poi import nav_to_poi

class ConcurrenceTime(smach.StateMachine):
    """ConcurrenceTime State.

    Use this StateMachine to execute a State with a time limit.

    All the states that you use must accept the preempted. 
    
    Example of use:
        STATES = [Sleeper(30)]
        STATE_NAMES = ["sleep"]
        INPUTS = ['sleep_time']
            
        smach.StateMachine.add('Conc_time',
                            ConcurrenceTime(state_name=STATE_NAMES, states=STATES, inputs=INPUTS, timeout=5),
                            transitions={
                                         'succeeded': 'succeeded', 
                                         'aborted': 'aborted', 'time_ends':'say_time_out'})

    """
    
    def __init__(self, states=[], state_name=[], inputs=[], outputs=[], timeout=0):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted',
                                                    'aborted', 'time_ends'])
        
        with self:
            def child_term_cb(outcome_map):
                #If time passed, we terminate all running states
                if outcome_map['TimeOut'] == 'succeeded':
                    rospy.loginfo('TimeOut finished')
                    return True
                counter = 0
                for state in states:
                    if outcome_map[str(counter) + state_name[counter]] == 'succeeded':
                        rospy.loginfo(str(counter) + state_name[counter] + ' Finished')
                        return True
                    counter += 1 

                #By default, just keep running
                return False
            
            def out_cb(outcome_map):
                if outcome_map['TimeOut'] == 'succeeded':
                    rospy.loginfo("Return time_ends")
                    return 'time_ends'   
                counter = 0
                for state in states:
                    if outcome_map[str(counter) + state_name[counter]] == 'succeeded':
                        return 'succeeded'
                    counter += 1 

                return 'aborted'
            
            rospy.logwarn(inputs)
            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted', 'time_ends'],
                                        default_outcome='succeeded',
                                        input_keys=inputs,
                                        output_keys=outputs,
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            counter = 0
            
            with sm_conc: 
                for state in states:
                    smach.Concurrence.add(str(counter) + state_name[counter], state)
                    counter += 1 
                    
                smach.Concurrence.add('TimeOut', TimeOut(timeout))
            
            smach.StateMachine.add(
                'Concurrence_with_time',
                sm_conc,
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'time_ends':'time_ends'}) 
            
          
def main():
    rospy.init_node('Conc_time_node')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
 
    with sm:
        sm.userdata.sleep_time = 10
        
        STATES = [Sleeper(30)]
        STATE_NAMES = ["sleep"]
        INPUTS = ["sleep_time"]
            
        smach.StateMachine.add('Conc_time',
                            ConcurrenceTime(state_name=STATE_NAMES, states=STATES, inputs=INPUTS, timeout=5),
                            transitions={
                                         'succeeded': 'succeeded', 
                                         'aborted': 'aborted', 'time_ends':'say_time_out'})
        # Say TimeOut 
        smach.StateMachine.add(
                'say_time_out',
                text_to_say("The time is finish"),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})
    sm.execute()
 
    rospy.spin()
 
if __name__ == '__main__':
    main()                    
               
            
