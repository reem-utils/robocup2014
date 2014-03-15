#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

26 Feb 2014
"""

import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.nav_to_coord import nav_to_coord
from face_states.drop_faces import drop_faces
from speech_states.say_sm import text_to_say
from save_face import SaveFaceSM
from search_faces import SearchFacesSM

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

class prepare_location(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name']) 

    def execute(self,userdata):
        userdata.nav_to_poi_name='find_me'
        return 'succeeded'
    
# Class that prepare the value need for nav_to_poi
class prepare_coord_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['face'], 
            output_keys=['nav_to_coord_goal']) 

    def execute(self,userdata):
        
        userdata.nav_to_coord_goal = [userdata.face.position.x, userdata.face.position.y, userdata.face.position.z]

        return 'succeeded'

class prepare_say_found(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name'],
                                output_keys=['tts_text','tts_wait_before_speaking'])

    def execute(self, userdata):

        userdata.tts_text = "I found you, " + userdata.name + "!"
        userdata.tts_wait_before_speaking = 0

        return 'succeeded'


class FindMeSM(smach.StateMachine):
    """
    Executes a SM that does the test find me and go over there.
    The robot recognize one TC and it must recognize this person inside a room.
    When it do it, it must go to the side that the TC indicates. 


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
            
            # Params for drop faces - always will be the same
            self.userdata.name_database = "find_me_database"
            self.userdata.purgeAll = True
            
            # Init the state machine - Drop faces in the database
            smach.StateMachine.add(
                'init_database',
                drop_faces(),
                remapping={'name':'name_database'},
                transitions={'succeeded': 'save_face', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Save the data from the TC
            # We need one state machine that listen the name and then learn the face
            smach.StateMachine.add(
                'save_face',
                SaveFaceSM(),
                transitions={'succeeded': 'prepare_location', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Prepare the info to go to the room
            smach.StateMachine.add(
                'prepare_location',
                prepare_location(),
                transitions={'succeeded': 'go_location', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Go to the location
            smach.StateMachine.add(
                'go_location',
                nav_to_poi(),
                transitions={'succeeded': 'search_faces', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
           
            # Go around the room, it the robot find the TC return success
            smach.StateMachine.add(
                'search_faces',
                SearchFacesSM(),
                transitions={'succeeded': 'prepare_coord_person', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Go to the person - We assume that find person will return the position for the person
            smach.StateMachine.add(
                'prepare_coord_person',
                prepare_coord_person(),
                transitions={'succeeded': 'go_to_person', 'aborted': 'aborted', 
                'preempted': 'preempted'})                    
            
            smach.StateMachine.add(
                'go_to_person',
                nav_to_coord(),
                transitions={'succeeded': 'prepare_say_found', 'aborted': 'aborted', 
                'preempted': 'preempted'})                    

            # Say "I found you!"
            smach.StateMachine.add(
                'prepare_say_found',
                prepare_say_found(),
                transitions={'succeeded': 'say_found', 'aborted': 'aborted', 'preempted': 'preempted'})

            smach.StateMachine.add(
                'say_found',
                text_to_say(),
                transitions={'succeeded': 'gesture_recognition', 'aborted': 'aborted'})

            # Recognize the direction
            smach.StateMachine.add(
                'gesture_recognition',
                DummyStateMachine(),
                transitions={'succeeded': 'go_side', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

            # Go to the direction
            smach.StateMachine.add(
                'go_side',
                DummyStateMachine(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})    

           

