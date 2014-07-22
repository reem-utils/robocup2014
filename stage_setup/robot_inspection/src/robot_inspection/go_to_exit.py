#! /usr/bin/env python
"""
Created on 21/07/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
    
from save_position import SavePosition
from check_button import CheckButton
from navigation_states.nav_to_poi import nav_to_poi

class prepareData(smach.State):
    
    def __init__(self, poi_name):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_poi_name'], output_keys=['nav_to_poi_name'])
        self.poi_name = poi_name
        
    def execute(self, userdata):
           
        if not self.poi_name and not userdata.nav_to_poi_name:
            rospy.logerr("Poi_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.nav_to_poi_name = self.poi_name if self.poi_name else userdata.nav_to_poi_name   
        
        return 'succeeded'
    
def child_term_cb(outcome_map):
    
    #If time passed, we terminate all running states
    if outcome_map['Go_to_poi'] == 'succeeded':
        rospy.loginfo('Go_to_poi finished')
        return True
    if outcome_map['check_button'] == 'end':
        rospy.loginfo('Check_button Finished')
        return True
    if outcome_map['save_position'] == 'succeeded':
        rospy.loginfo('Save_position Finished')
        return False
    #By default, just keep running
    return False
    
def out_cb(outcome_map):
    if outcome_map['Go_to_poi'] == 'succeeded':
        return 'succeeded'   
    elif outcome_map['check_button'] == 'end':
        return 'succeeded'
    elif outcome_map['save_position'] == 'succeeded':
        return 'succeeded'
    else:
        return 'aborted'
    
class GoToExit(smach.StateMachine):
    """
    This state machine goes to exit in robot inspection. 
    It saves the actual position and wait for the button emergency.   
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    """
    def __init__(self, poi_name=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['nav_to_poi_name'],
                                     output_keys=['pose_current'])
            
        with self:
            smach.StateMachine.add('PrepareData',
                                   prepareData(poi_name),
                                   transitions={'succeeded':'Concurrence', 'aborted':'aborted'})
            
            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                        default_outcome='succeeded',
                                        input_keys=['nav_to_poi_name'],
                                        output_keys=['standard_error', 'pose_current'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            with sm_conc: 
                smach.Concurrence.add('Go_to_poi', nav_to_poi())
                
                smach.Concurrence.add('check_button', CheckButton())
                
                smach.Concurrence.add('save_position', SavePosition())
            
            smach.StateMachine.add(
                'Concurrence',
                sm_conc,
                transitions={'succeeded': 'succeeded', 'aborted': 'Concurrence'}) 
           
class print_vars(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['pose_current'],
                                output_keys=['pose_current'])


    def execute(self, userdata):
        rospy.loginfo(userdata.pose_current)
        return 'succeeded'
    
def main():
    """
        Test the timeout and the sleeper
    """
    
    rospy.init_node('basic_functionalities')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
        sm.userdata.nav_to_poi_name = 'exit_door'
        sm.userdata.pose_current = None
        
        smach.StateMachine.add(
            'go_to_exit',
            GoToExit(),
            transitions={'succeeded': 'print_vars', 'aborted': 'aborted'})  
   
        smach.StateMachine.add(
            'print_vars',
            print_vars(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'}) 
       

    sm.execute()



if __name__ == '__main__':
    main()