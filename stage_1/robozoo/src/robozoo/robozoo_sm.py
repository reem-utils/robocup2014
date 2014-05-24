#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author:  Cristina De Saint Germain
@email: crsaintc8@gmail.com

@author:  Sergi Xavier Ubach Pallàs
@email: sxubach@gmail.com

12 Mar 2014
"""

import rospy
import smach

from manipulation_states.play_motion_sm import play_motion_sm

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class RoboZooSM(smach.StateMachine):
    """
    Executes a SM that does the robozoo
    The robot dances for an hour. It does different dances in a loop. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            
            # Do the Macarena Dance
            smach.StateMachine.add(
                'do_macarena',
                #DummyStateMachine(),
                play_motion_sm(motion='macarena'),
                transitions={'succeeded': 'do_YMCA', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Do the YMCA Dance
            smach.StateMachine.add(
                'do_YMCA',
                play_motion_sm(motion='ymca'),
                transitions={'succeeded': 'do_robot', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Do the Robot Dance
            smach.StateMachine.add(
                'do_robot',
                play_motion_sm('robot'),
                transitions={'succeeded': 'succeed', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
