#! /usr/bin/env python

'''
Created on 21/07/2014

@author: Cristina De Saint Germain
'''

import rospy
import smach
from navigation_states.get_current_robot_pose import get_current_robot_pose
from util_states.sleeper import Sleeper
from emergency_button import emergency_button

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class save_robot_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['current_robot_pose'],
                                output_keys=['pose_current'])


    def execute(self, userdata):
        userdata.pose_current = userdata.current_robot_pose
        #rospy.loginfo(userdata.current_robot_pose)
        if self.preempt_requested():
                rospy.logwarn('PREEMPT REQUESTED -- Returning Preempted in save_robot State')
                return 'preempted'
        return 'succeeded'

class SavePosition(smach.StateMachine):
    """
    This state machine saves the actual position every second.  
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=[],
                                     output_keys=['pose_current'])

        with self:
            
            # Calculate the actual position
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pose(),
                transitions={'succeeded': 'save_robot_position', 'aborted': 'get_actual_pos', 'preempted': 'succeeded'})

            # Save position
            smach.StateMachine.add(
                'save_robot_position',
                save_robot_position(),
                transitions={'succeeded': 'sleep_state', 'aborted': 'save_robot_position', 'preempted':'preempted'})
            
            # Sleep
            smach.StateMachine.add(
                'sleep_state',
                Sleeper(2),
                transitions={'succeeded': 'check_button', 'aborted': 'check_button', 'preempted':'preempted'})
            
            # Check if the button is pressed
            smach.StateMachine.add(
                'check_button',
                emergency_button(),
                transitions= {'succeeded':'succeeded', 'aborted':'get_actual_pos', 'preempted':'preempted'})
            
def main():
    rospy.init_node('save_position_node')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'SavePosition',
            SavePosition(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})    

    sm.execute()


if __name__ == '__main__':
    main()
