#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 15 12:00:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
import smach_ros
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from emergency_situation.emergency_situation_sm import emergency_situation_sm
#from emergency_situation.Get_Person_Desired_Object import Get_Person_Desired_Object
#from emergency_situation.Save_People_Emergency import Save_People_Emergency

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class emergency_situation_sm(smach.StateMachine):

    def __init__(self):
        sm = smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:           
            # We prepare the information to go to the init door
            self.userdata.manip_motion_to_play = 'home'
            self.userdata.manip_time_to_play = 4.0
            smach.StateMachine.add(
                'Arms_Home',
                play_motion_sm(),
                transitions={'succeeded':'Prepare_TTS_1', 'aborted':'Prepare_TTS_1', 'preempted':'Prepare_TTS_1'})
            
            self.userdata.tts_wait_before_speaking = 0
            smach.StateMachine.add(
                'Prepare_TTS_1',
                prepare_tts("I am ready to save people"),
                transitions={'succeeded':'Say_Ready', 'aborted':'Say_Ready', 'preempted':'Say_Ready'})

            smach.StateMachine.add(
                'Say_Ready',
                text_to_say(),
                transitions={'succeeded':'Prepare_Door_Out_Arena', 'aborted':'Prepare_Door_Out_Arena', 'preempted':'Prepare_Door_Out_Arena'})

            #TODO: Define the poi for the output of the room 
            # Pre: The robot should be in front of the Arena door (maybe we should change this, depending on the conditions)
            smach.StateMachine.add(
                'Prepare_Door_Out_Arena',
                prepare_poi_emergency('arena_door_out'),
                transitions={'succeeded':'Enter_Room_Arena', 'aborted':'Enter_Room_Arena', 'preempted':'Enter_Room_Arena'})

            smach.StateMachine.add(
                'Enter_Room_Arena',
                EnterRoomSM(),
                transitions={'succeeded':'Prepare_Poi_Emergency_1', 'aborted':'Prepare_Poi_Emergency_1', 'preempted':'Prepare_Poi_Emergency_1'})

            #TODO: Define the name of the room to enter (defined by the OC)
            #If Aborted (not supposed to), retry?
            smach.StateMachine.add(
                'Prepare_Poi_Emergency_1',
                prepare_poi_emergency('emergency_room'),
                transitions={'succeeded':'Go_to_emergency_room', 'aborted':'Go_to_emergency_room', 'preempted':'Go_to_emergency_room'})
            smach.StateMachine.add(
                'Go_to_emergency_room',
                nav_to_poi(),
                transitions={'succeeded':'Search_Person', 'aborted':'Go_to_emergency_room', 'preempted':'Go_to_emergency_room'})

            # Userdata output keys:
            #  - emergency_person_location: PoseStamped/Geometry_msg (?)
            #   Another state will be needed (maybe) to remap
            # No need of face_recognition
            # What if person not found? Re-search?
           # smach.StateMachine.add(
           #     'Search_People',
           #     Search_People_Emergency(),
           #     transitions={'succeeded':'Save_Person', 'aborted':'Search_Person', 'preempted':'Search_Person'})

            # Userdata input:
            # person_location: PoseStamped (?)
            # It is a SuperStateMachine (contains submachines) with these functionalities (draft):
            # 1. Go to Person location
            # 2. Ask Status
            # 3. Register position
            # 4. Save info
            # What to do if fail?
            smach.StateMachine.add(
                'Save_Person',
                Save_People_Emergency(),
                transitions={'succeeded':'Get_Person_Desired_Object', 'aborted':'Get_Person_Desired_Object', 'preempted':'Get_Person_Desired_Object'})

            # The functionalities of this SuperSM are:
            # 1. Ask the person what to fetch
            # 2. Go and grab the object  --> Similar with Pick-and-Place
            #   2.1. Go to room
            #   2.2. Find Object 
            #   2.3. Go to Object
            #   2.4. Grab Object
            #   2.5. Go to person
            #   2.6. Give object --> Ungrab
            #--> Database of objects and their location
            #                           --> Manip/Grab 
            # 

            smach.StateMachine.add(
                'Get_Person_Desired_Object',
                Get_Person_Desired_Object(),
                transitions={'succeeded':'Prepare_Door_Out_Arena_2', 'aborted':'Prepare_Door_Out_Arena_2', 'preempted':'Prepare_Door_Out_Arena_2'})

            #TODO: Define Entry room POI: userdata.nav_poi (?)
            #Retrying to go to entry_door until is succeeded
            smach.StateMachine.add(
                'Prepare_Door_Out_Arena_2',
                prepare_poi_emergency('arena_door_out'),
                transitions={'succeeded':'Go_to_Entry_Door', 'aborted':'Go_to_Entry_Door', 'preempted':'Go_to_Entry_Door'})
            smach.StateMachine.add(
                'Go_to_Entry_Door',
                nav_to_poi(),
                transitions={'succeeded':'Wait_for_Ambulance_Person', 'aborted':'Go_to_Entry_Door', 'preempted':'Go_to_Entry_Door'})

            #What is Wait for Ambulance or People Mean? Person detection?
#            smach.StateMachine.add(
#                'Wait_for_Ambulance_Person',
#                Wait_for_Ambulance_Person(),
#                transitions={'succeeded':'Prepare_TTS_2', 'aborted':'Go_to_Entry_Door', 'preempted':'Go_to_Entry_Door'})
            
            smach.StateMachine.add(
                'Prepare_TTS_2',
                prepare_tts("Please Follow Me, I will guide you to the emergency"),
                transitions={'succeeded':'Say_Ambulance', 'aborted':'Say_Ambulance', 'preempted':'Say_Ambulance'})
            smach.StateMachine.add(
                'Say_Ambulance',
                text_to_say(),
                transitions={'succeeded':'Prepare_Poi_Emergency_2', 'aborted':'Prepare_Poi_Emergency_2', 'preempted':'Prepare_Poi_Emergency_2'})
            #TODO: Define the name of the room to enter (defined by the OC)
            #If Aborted (not supposed to), retry?
            smach.StateMachine.add(
                'Prepare_Poi_Emergency_2',
                prepare_poi_emergency('emergency_room'),
                transitions={'succeeded':'Go_to_emergency_room_2', 'aborted':'Go_to_emergency_room_2', 'preempted':'Go_to_emergency_room_2'})
            smach.StateMachine.add(
                'Go_to_emergency_room_2',
                nav_to_poi(),
                transitions={'succeeded':'DummyStateMachine', 'aborted':'DummyStateMachine', 'preempted':'DummyStateMachine'})

            smach.StateMachine.add(
                'Dummy_Wait',
                DummyStateMachine(),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
    
        sis = smach_ros.IntrospectionServer(
            'emergency_situation_introspection', sm, '/SM_ROOT')
        sis.start()
    
        sm.execute()
    
        rospy.spin()
        sis.stop()
