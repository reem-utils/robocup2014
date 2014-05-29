#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat May 29 13:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
from navigation_states.get_current_robot_pose import get_current_robot_pose
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from util_states.topic_reader import topic_reader
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

#from emergency_situation.GeneratePDF_State import GeneratePDF_State

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[]) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(1)
        return 'succeeded'

class Select_Possible_Poi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'None'], 
                                input_keys=[],
                                output_keys=['standard_error', 'nav_to_poi_name'])
    def execute(self, userdata):
        
        return 'succeeded'


class Search_Emergency_Wave_Room_Change(smach.StateMachine):
    """
    Executes a SM that navigates to each room to detect a wave gesture, in order to find the person in emergency state.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input_keys:
        None
    Output Keys:
        @key poi_location: location of the emergency room
        @key wave_position 
        @key wave_yaw_degree
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'],
                                    input_keys=[],
                                    output_keys=['poi_location'])

        
        with self:           
            self.userdata.emergency_location = []
            self.userdata.tts_lang = 'en_US'
            self.userdata.tts_wait_before_speaking = 0
    
            smach.StateMachine.add(
                'Save_Info',
                DummyStateMachine(),
                #GeneratePDF_State(),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})

