#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 30 12:30:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from util_states.topic_reader import topic_reader
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gesture_states.gesture_detection_sm import gesture_detection_sm
from gesture_detection_mock.msg import Gesture

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(1)
        return 'succeeded'

class prepare_tts(smach.State):
    def __init__(self, tts_text_phrase):
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted', 'preempted'], 
            output_keys=['tts_text']) 
        self.tts_text_phrase_in = tts_text_phrase
    def execute(self, userdata):
        userdata.tts_text = self.tts_text_phrase_in
        return 'succeeded'

class Analyze_Wave(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['gesture_detected'], output_keys=['person_location'])
    def execute(self, userdata):
        if (userdata.gesture_detected.Gesture_name.data == "Wave"):
            userdata.person_location = userdata.gesture_detected.gesture_position #Type: geometry_msgs/Pose
            return 'succeeded'
        else:
            userdata.person_location = None
            return 'aborted'

class Search_People_Emergency(smach.StateMachine):
    """
    Executes a SM that does the Emergency Situation's Search People SM.
    Pre: The robot has to be in the same room as the peson.
    It is a SuperStateMachine (contains submachines) with these functionalities (draft):
    1. Wave detector
    2. Face detector

    What to do if fail?

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input_keys:
        none

    Output Keys:
        @key: person_location: person's location (Geometry or PoseStamped)
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'],
                                    output_keys=['person_location'])

        with self:           
            self.userdata.emergency_location = []

            # Some dummy TTS stuff
            self.userdata.tts_lang = 'en_US'
            self.userdata.tts_wait_before_speaking = 0
            smach.StateMachine.add(
                'Prepare_Say_Searching',
                prepare_tts('Where are you? Give me signals, please.'),
                transitions={'succeeded':'Say_Search', 'aborted':'Say_Search', 'preempted':'Say_Search'})
            smach.StateMachine.add(
                'Say_Search',
                text_to_say(),
                transitions={'succeeded':'Detect_Wave', 'aborted':'Detect_Wave', 'preempted':'Detect_Wave'})

            # Search for a Wave Gesture
            # Output_keys: gesture_detected: type Gesture
            smach.StateMachine.add(
                'Detect_Wave',
                gesture_detection_sm(),
                transitions={'succeeded':'Analyze_Wave', 'aborted':'Analyze_Wave', 'preempted':'Analyze_Wave'})
            smach.StateMachine.add(
                'Analyze_Wave',
                Analyze_Wave(),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
