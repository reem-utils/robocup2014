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
from gesture_states.gesture_recognition import GestureRecognition 
from gesture_states.wave_detection_sm import WaveDetection
from gesture_detection_mock.msg import Gesture
from navigation_states.get_current_robot_pose import get_current_robot_pose
from emergency_situation.Search_Emergency_Wave_Room_Change_SM import Search_Emergency_Wave_Room_Change


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

class prepare_go_to_wave(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['succeeded', 'aborted', 'preempted'],
                            input_keys=['wave_position','wave_yaw_degree', 'nav_to_coord_goal'],
                            output_keys=['standard_error', 'nav_to_coord_goal'])
    def execute(self, userdata):
        userdata.nav_to_coord_goal = [userdata.wave_position.point.x, userdata.wave_position.point.y, 
                                            userdata.wave_yaw_degree]
        
        #userdata.nav_to_coord_goal.y = userdata.gesture_detected.gesture_position.position.y
        #userdata.nav_to_coord_goal.yaw  = userdata.gesture_detected.gesture_position.orientation.w
        return 'succeeded'        

class Analyze_Wave(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['gesture_detected'], 
                             output_keys=['person_location'])
    def execute(self, userdata):
        if (userdata.gesture_detected.Gesture_name.data == "Wave"):
            userdata.person_location = userdata.gesture_detected.gesture_position #Type: geometry_msgs/Pose
            return 'succeeded'
        else:
            userdata.person_location = None
            return 'aborted'
class PoseToArray(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['person_location', 'current_robot_yaw'],
                             output_keys=['person_location_coord'])
    def execute(self, userdata):
        userdata.person_location_coord = [userdata.person_location.pose.position.x, 
                                          userdata.person_location.pose.position.y,
                                          userdata.current_robot_yaw]
        return 'succeeded'

class Search_People_Emergency(smach.StateMachine):
    """
    Executes a SM that does the Emergency Situation's Search People SM.
    Pre: The robot has to be in the same room as the person.
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
                                    output_keys=['person_location', 'person_location_coord'])

        with self:           
            self.userdata.emergency_location = []

            self.userdata.tts_lang = 'en_US'
            self.userdata.tts_wait_before_speaking = 0

#             smach.StateMachine.add(
#                 'Say_Search',
#                 text_to_say('Where are you? Give me signals, please.'),
#                 transitions={'succeeded':'Gesture_Recognition', 'aborted':'Gesture_Recognition', 'preempted':'Gesture_Recognition'})
# 
#             # Search for a Wave Gesture
#             # Output_keys: gesture_detected: type Gesture
#             self.userdata.nav_to_coord = [0, 0, 0]
#             #Look Down
#             #Move head right/left
#             smach.StateMachine.add(
#                 'Gesture_Recognition',
#                 WaveDetection(),
#                 transitions={'succeeded':'Say_Search','aborted':'Gesture_Recognition', 'preempted':'Gesture_Recognition'})
            
            smach.StateMachine.add(
                'Search_Person_Room_by_Room',
                Search_Emergency_Wave_Room_Change(),
                transitions={'succeeded':'Say_Search', 'aborted':'Say_No_People_Found', 'preempted':'Say_Search'})
            
            #This is the worst-case scenario: The person could not be found, so we are losing an important amount of points
            smach.StateMachine.add(
                'Say_No_People_Found',
                text_to_say("I could not find any person in an emergency situation, sorry. Can you come to me?"),
                transitions={'succeeded':'Register_Position', 'aborted':'aborted'})
            
            smach.StateMachine.add(
                'Say_Search',
                text_to_say('Oh! I have found you at last. Let me help you.'),
                transitions={'succeeded':'Prepare_Go_To_Wave', 'aborted':'Prepare_Go_To_Wave', 'preempted':'Say_Search'})
            
            smach.StateMachine.add(
                'Prepare_Go_To_Wave',
                prepare_go_to_wave(),
                transitions={'succeeded':'Say_Go_to_Wave', 'aborted':'Say_Go_to_Wave', 'preempted':'Say_Go_to_Wave'})
            
            smach.StateMachine.add(
                'Say_Go_to_Wave',
                text_to_say("I'm coming!"),
                transitions={'succeeded':'Go_to_Wave', 'aborted':'Gesture_Recognition', 'preempted':'Gesture_Recognition'})
            
            smach.StateMachine.add(
                'Go_to_Wave',
                nav_to_coord('/base_link'),
                transitions={'succeeded':'Say_Arrive_to_Wave', 'aborted':'Go_to_Wave', 'preempted':'Go_to_Wave'})
            
            smach.StateMachine.add(
                'Say_Arrive_to_Wave',
                text_to_say("I'm here!"),
                transitions={'succeeded':'Register_Position', 'aborted':'Register_Position', 'preempted':'Register_Position'})
            
            smach.StateMachine.add(
                'Register_Position',
                get_current_robot_pose(),
                transitions={'succeeded':'TreatPoseForCoord', 'aborted':'Register_Position', 'preempted':'Register_Position'},
                remapping={'current_robot_pose':'person_location'})
            smach.StateMachine.add(
                'TreatPoseForCoord',
                PoseToArray(),
                transitions={'succeeded':'succeeded', 'aborted':'Register_Position', 'preempted':'Register_Position'})
            