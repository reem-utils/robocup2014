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
from emergency_situation.Get_Person_Desired_Object import Get_Person_Desired_Object
from emergency_situation.Save_People_Emergency import Save_People_Emergency
from emergency_situation.Search_People_Emergency import Search_People_Emergency
from emergency_situation.Ambulance_Detect_And_Go import Ambulance_Detect_And_Go

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


# Class that prepare the value need for nav_to_poi
class prepare_poi_emergency(smach.State):
    def __init__(self, poi_type='entry_door_exit'):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 
        self.poi_type_in = poi_type
    def execute(self,userdata):
        if self.poi_type_in == 'entry_door_exit':
            userdata.nav_to_poi_name='entry_door_exit'    
        elif self.poi_type_in == 'emergency_room':
            userdata.nav_to_poi_name='emergency_room'

        return 'succeeded'


# Class that prepare the value need for nav_to_poi
class prepare_poi_person_emergency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['person_location_coord'], 
            output_keys=['nav_to_coord_goal']) 
    def execute(self,userdata):
        rospy.loginfo('PersonLOcationCooordddddd :::::::: ' + str(userdata.person_location_coord))
        userdata.nav_to_coord_goal = userdata.person_location_coord 

        return 'succeeded'



class emergency_situation_sm(smach.StateMachine):
    """
    Executes a SM that does the Emergency Situation.
    - Enter Apartment... (do we need this?)
    - It goes to a specific room (indicated by the OC 1h before)
    - Waits a random time: 30 - 60 seconds
    - Find person: Gesture recognition, because the person is not standing
    - Go to person and memorize position
    - Ask the person a question: e.g. What is your status? How are you?
    - Report the answer to the ambulance
    - Ask the person about the item to get: e.g. What do you need?
    - Find the object
    - Grab the object
    - Bring the object to the person
    - Go to the apartment's entry
    - Guide the helper to the person's position
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input_keys:
    @key: emergency_room: name of the room where the emergency is located

    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        sm = smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:           
            smach.StateMachine.add(
                'Say_Init',
                text_to_say("Initializing and Preparing Emergency Test. Let's go!"),
                transitions={'succeeded':'Arms_Home'})
            # Put the arms at the 'Home' position, in order improve the security
            # We prepare the information to go to the init door
            self.userdata.manip_motion_to_play = 'home'
            self.userdata.skip_planning = False
            smach.StateMachine.add(
                'Arms_Home',
                play_motion_sm(),
                transitions={'succeeded':'Say_Ready', 'aborted':'Say_Ready', 'preempted':'Say_Ready'})
            
            #Prepare the data for the speech: language and time to wait
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_lang = 'en_US'

            smach.StateMachine.add(
                'Say_Ready',
                text_to_say('Emergency Situation Test is going to Start. Now I am going to enter the Room to help people in need.'),
                transitions={'succeeded':'Enter_Room_Arena', 'aborted':'Enter_Room_Arena', 'preempted':'Enter_Room_Arena'})

            # Pass the entry room
            # TODO: Define the poi for the output of the room 
            # Pre: The robot should be in front of the Arena door (maybe we should change this, depending on the conditions)
            smach.StateMachine.add(
                'Enter_Room_Arena',
                EnterRoomSM('entry_door_exit'),
                transitions={'succeeded':'Say_Enter_Successful', 'aborted':'Say_Ready', 'preempted':'Search_Person'})
            smach.StateMachine.add(
                'Say_Enter_Successful',
                text_to_say('I have successfully entered the apartment. Now I am going to find and rescue the person in emergency.'),
                transitions={'succeeded':'Search_Person', 'aborted':'Search_Person', 'preempted':'Search_Person'})

#            The Location for the Emergency is not known --> Search for the emergency situation 

#             smach.StateMachine.add(
#                 'Prepare_Poi_Emergency_1',
#                 prepare_poi_emergency('emergency_room'),
#                 transitions={'succeeded':'Go_to_emergency_room'})
#             smach.StateMachine.add(
#                 'Go_to_emergency_room',
#                 nav_to_poi(),
#                 transitions={'succeeded':'Say_Search_Person', 'aborted':'Go_to_emergency_room', 'preempted':'Go_to_emergency_room'})

            # Userdata output keys:
            #  - person_location: PoseStamped/-->Pose<-- (?)
            #   Another state will be needed (maybe) to remap
            # No need of face_recognition
            # What if person not found? Re-search?
#             smach.StateMachine.add(
#                 'Say_Search_Person',
#                 text_to_say('Successfully entered the room. Now I am going to search for a person in an emergency situation.'),
#                 transitions={'succeeded':'Search_Person', 'aborted':'Search_Person', 'preempted':'Search_Person'})
            smach.StateMachine.add(
                'Search_Person',
                Search_People_Emergency(),
                transitions={'succeeded':'Save_Person', 'aborted':'Search_Person', 'preempted':'Search_Person'})

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
                transitions={'succeeded':'Get_Person_Desired_Object', 'aborted':'Save_Person', 'preempted':'Save_Person'})

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
                        transitions={'succeeded':'Ambulance_SM', 'aborted':'Get_Person_Desired_Object', 'preempted':'Ambulance_SM'})

            #TODO: Define Entry room POI: userdata.nav_poi (?)
            #Retrying to go to entry_door until is succeeded
            smach.StateMachine.add(
                                   'Ambulance_SM',
                                   Ambulance_Detect_And_Go(),
                                   transitions={'succeeded':'Final_State', 'aborted':'aborted', 'preempted':'preempted'})
            
            smach.StateMachine.add(
                                   'Final_State',
                                   text_to_say('Emergency Situation Test 2014 Finished Successfully.'),
                                   transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
            
        