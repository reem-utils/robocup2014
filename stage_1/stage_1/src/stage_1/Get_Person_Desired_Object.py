#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 16 11:30:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM

from manipulation_states.play_motion_sm import play_motion_sm

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'

# Class that prepare the value need for nav_to_poi
class prepare_poi_person_emergency(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['person_location'], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata, poi_type='arena_door_out'):
        userdata.nav_to_poi_name = userdata.person_location

        return 'succeeded'

class prepare_tts(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted', 'preempted'], 
            output_keys=['tts_text']) 
    def execute(self, userdata, tts_text_phrase=''):
        userdata.tts_text = tts_text_phrase

        return 'succeeded'


class Get_Person_Desired_Object(smach.StateMachine):
    """
    Executes a SM that does the Emergency Situation's Save People SM.
    It is a SuperStateMachine (contains submachines) with these functionalities (draft):
    # The functionalities of this SuperSM are:
    # 1. Ask the person what to fetch
    # 2. Go and grab the object  --> Similar with Pick-and-Place
    #   2.1. Go to room
    #   2.2. Find Object 
    #   2.3. Go to Object
    #   2.4. Grab Object
    #   2.5. Go to person
    #   2.6. Give object --> Ungrab
    #                    --> Database of objects and their location
    #                    --> Manip/Grab 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input_keys:
    @key: emergency_person_location: person's location (Geometry or PoseStamped)

    Output Keys:
        none
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'],
                                    input_keys=['person_location'])

        with self:           
            self.userdata.emergency_location = []

            userdata.tts_wait_before_speaking = 0
            smach.StateMachine.add(
                'Prepare_Say_Rescue',
                prepare_tts('I am going to rescue you!'),
                transitions={'succeeded':'Say_Rescue', 'aborted':'Say_Rescue', 'preempted':'Say_Rescue'})

            smach.StateMachine.add(
                'Say_Rescue',
                text_to_say(),
                transitions={'succeeded':'Prepare_Go_To_Person', 'aborted':'Prepare_Go_To_Person', 'preempted':'Prepare_Go_To_Person'})

            smach.StateMachine.add(
                'Prepare_Go_To_Person',
                prepare_poi_person_emergency(),
                transitions={'succeeded':'Go_To_Person', 'aborted':'Go_To_Person', 'preempted':'Go_To_Person'})

            smach.StateMachine.add(
                'Go_To_Person',
                nav_to_poi(),
                transitions={'succeeded':'Prepare_Ask_Status', 'aborted':'Prepare_Ask_Status', 'preempted':'Prepare_Ask_Status'})

            #It should be Speak Recognition
            smach.StateMachine.add(
                'Prepare_Ask_Status',
                prepare_tts('Are you Ok?'),
                transitions={'succeeded':'Ask_Status', 'aborted':'Ask_Status', 'preempted':'Ask_Status'})

            smach.StateMachine.add(
                'Ask_Status',
                text_to_say(),
                transitions={'succeeded':'Register_Position', 'aborted':'Register_Position', 'preempted':'Register_Position'})

            #Register Position --> TODO? or done?
            #Output keys: topic_output_msg & standard_error
            userdata.position_stamped = ''
            smach.StateMachine.add(
                'Register_Position',
                topic_reader('amcl_pose', PoseStamped, 30),
                transitions={'succeeded':'Save_Info', 'aborted':'Save_Info', 'preempted':'Save_Info'},
                remapping={'topic_output_msg':'position_stamped', 'standard_error':'standard_error'})

            smach.StateMachine.add(
                'Save_Info',
                Save_Info(),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
            