#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat March 16 11:30:00 2013

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
from smach import Concurrence
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from speech_states.ask_question import AskQuestionSM
from speech_states.listen_to import ListenToSM
from manipulation_states.play_motion_sm import play_motion_sm
from manipulation_states.move_hands_form import move_hands_form
from manipulation_states.ask_give_object_grasping import ask_give_object_grasping
from util_states.topic_reader import topic_reader
from geometry_msgs.msg import PoseStamped
from manipulation_states.grasp_time_out import grasping_with_timeout
import time

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

time_First = True

import random

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[]) 

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
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

class prepare_tts(smach.State):
    def __init__(self,tts_text_phrase=''):
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted', 'preempted'], 
            output_keys=['tts_text']) 
        self.tts_text_phrase_in = tts_text_phrase
    def execute(self, userdata):
        userdata.tts_text = self.tts_text_phrase_in

        return 'succeeded'

class Process_Tags(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=["asr_userSaid","asr_userSaid_tags"],
                                output_keys=['object_to_grasp'])

    def execute(self, userdata):
        if(userdata.asr_userSaid.find("water")):
            userdata.object_to_grasp = 'water'
            rospy.loginfo(userdata.asr_userSaid)
            return 'succeeded'
        elif(userdata.asr_userSaid.find("kit")):
            userdata.object_to_grasp = 'First Aid Kit'
            rospy.loginfo(userdata.asr_userSaid)
            return 'succeeded'
        elif(userdata.asr_userSaid.find("phone")):
            userdata.object_to_grasp = 'Cell phone'
            rospy.loginfo(userdata.asr_userSaid)
            return 'succeeded'
        
#         tags = [tag for tag in userdata.asr_answer_tags if tag.key == 'object']
#         if tags:
#             name = tags[0].value
#             userdata.object_to_grasp = name
        return 'aborted'

class Time_State(smach.State):
    def __init__(self, ttl=20.0):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'time_out', 'preempted'],
                             input_keys=['time_grasp'],
                             output_keys=['time_grasp'])
        self.time_to_live_grasp = ttl
    def execute(self, userdata):
        if time_First :
            time_First = False
            userdata.time_grasp = time.time()
        
        if (time.time() - self.time_to_live_grasp) > userdata.time_grasp :
            return 'time_out'
        else:
            return 'preempted' 
        

def child_term_cb(outcome_map):
    if outcome_map['Time_State'] == 'time_out':
        return True
    
    if outcome_map['Find_and_grab_object'] == 'succeeded':
        return True
    
    return False
        
def out_cb(outcome_map):
    if outcome_map['Time_State'] == 'time_out':
        return 'time_out'
    if outcome_map['Find_and_grab_object'] == 'succeeded':
        return 'succeeded'
    return 'aborted'
    
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
                                    input_keys=['person_location', 'person_location_coord'])

        with self:           
            self.userdata.emergency_location = []
            self.userdata.tts_lang = 'en_US'
            self.userdata.tts_wait_before_speaking = 0
            smach.StateMachine.add(
                                   'Ask_Question',
                                   text_to_say(text='What would you like me to bring?'),
                                   transitions={'succeeded':'Listen_Question', 'aborted': 'Ask_Question', 'preempted':'Listen_Question'})
            # Ask for the object to bring to the person
            # Output: 
            #   - string 'userSaid'
            #   - actiontag[] 'asr_userSaid_tags'
            # TODO: grammar for the Emergency Situation -- Get_Person_Desired_Object
            smach.StateMachine.add(
                'Listen_Question',
                ListenToSM(grammar='robocup/emergency'),
                transitions={'succeeded':'Process_Tags', 'aborted':'Ask_Question', 'preempted':'Ask_Question'})

            # Get the output from AskQuestionSM, process it, and search in the yaml file for the location of the object asked 
            # Input keys: actiontag[] 'asr_userSaid_tags'
            # Output keys: object
            smach.StateMachine.add(
                'Process_Tags',
                Process_Tags(),
                transitions={'succeeded':'Say_go_Kitchen', 'aborted':'Ask_Question', 'aborted':'Ask_Question'})
            smach.StateMachine.add(
                'Say_go_Kitchen',
                text_to_say('I am Going to the Kitchen for an object, Stay Here until I give you the object'),
                transitions={'succeeded':'Go_To_Object_Place', 'aborted':'Go_To_Object_Place', 'aborted':'Go_To_Object_Place'})
            smach.StateMachine.add(
                'Go_To_Object_Place',
                nav_to_poi('kitchen'),
                transitions={'succeeded':'Say_got_to_Kitchen', 'aborted':'Grasp_fail_Ask_Person', 'preempted':'Grasp_fail_Ask_Person'})
            
            smach.StateMachine.add(
                'Say_got_to_Kitchen',
                text_to_say('I am in the Kitchen, I am going to grasp fail ask person'),
                transitions={'succeeded':'Grasp_fail_Ask_Person', 'aborted':'Grasp_fail_Ask_Person', 'aborted':'Grasp_fail_Ask_Person'})
            
            self.userdata.time_grasp = 0.0
            smach.StateMachine.add('Grasping_with_timeout',
                                   grasping_with_timeout(),
                                   transitions={'succeeded':'Prepare_Go_To_Person', 'time_out':'Grasp_fail_Ask_Person'})
#             sm_conc = smach.Concurrence(outcomes=['succeeded', 'time_out'],
#                                         default_outcome='succeeded',
#                                         input_keys=['object_to_grasp, time_grasp'],
#                                         child_termination_cb = child_term_cb,
#                                         outcome_cb = out_cb)
# 
#             with sm_conc:
#                 sm_conc.add(
#                     'Find_and_grab_object',
#                     #Find_and_grab_object(),
#                     DummyStateMachine())
#                 sm_conc.add(
#                             'Time_State',
#                             Time_State())
#                 
#             smach.StateMachine.add('GRASP_CONCURRENCE',
#                                    sm_conc,
#                                    transitions={'succeeded':'Prepare_Go_To_Person',
#                                                 'time_out':'Grasp_fail_Ask_Person'})
            #Find Object + Grab Object SM
#             smach.StateMachine.add(
#                 'Find_and_grab_object',
#                 #Find_and_grab_object(),
#                 DummyStateMachine(),
#                 transitions={'succeeded':'Grasp_fail_Ask_Person', 'aborted':'Grasp_fail_Ask_Person', 'preempted':'Grasp_fail_Ask_Person'})
            smach.StateMachine.add(
                'Grasp_fail_Ask_Person',
                ask_give_object_grasping(),
                transitions={'succeeded':'Rest_arm', 'aborted':'Rest_arm', 'preempted':'Rest_arm'})
            
            
            smach.StateMachine.add(
                           'Rest_arm',
                           play_motion_sm('rest_object_right'),
                           transitions={'succeeded':'Prepare_Go_To_Person', 'aborted':'Prepare_Go_To_Person', 'preempted':'Prepare_Go_To_Person'})
            
            smach.StateMachine.add(
                'Say_return_Person',
                text_to_say('I am preparing to go back to the person'),
                transitions={'succeeded':'Prepare_Go_To_Person', 'aborted':'Prepare_Go_To_Person', 'aborted':'Prepare_Go_To_Person'})
            
            #Go to person
            smach.StateMachine.add(
                'Prepare_Go_To_Person',
                prepare_poi_person_emergency(),
                transitions={'succeeded':'Go_To_Person', 'aborted':'Go_To_Person', 'preempted':'Go_To_Person'})
            
            #TODO: POI For Person in Emergency -- From SearchPeople SM - 
            smach.StateMachine.add(
                'Go_To_Person',
                #DummyStateMachine(),
                #nav_to_poi(),
                nav_to_coord('/map'),
                transitions={'succeeded':'Say_Give_Object', 'aborted':'Say_Give_Object', 'preempted':'Say_Give_Object'})
            smach.StateMachine.add(
                'Say_Give_Object',
                text_to_say('I am going to give you the Object you want.'),
                transitions={'succeeded':'Give_object_arm', 'aborted':'Give_object_arm', 'preempted':'Give_object_arm'})
            smach.StateMachine.add(
                                   'Give_object_arm',
                                   play_motion_sm('give_object_right'),
                                   transitions={'succeeded':'Give_Object', 'aborted':'Give_Object', 'preempted':'Give_Object'})
            #Give the grabbed object to the person
            smach.StateMachine.add(
                'Give_Object',
                #DummyStateMachine(),
                move_hands_form(hand_pose_name='pre_grasp', hand_side='right'),
                transitions={'succeeded':'Give_Object_2', 'aborted':'Give_Object', 'preempted':'Give_Object'})
            smach.StateMachine.add(
                'Give_Object_2',
                #DummyStateMachine(),
                move_hands_form(hand_pose_name='full_open', hand_side='right'),
                transitions={'succeeded':'Say_Rescue_stay', 'aborted':'Give_Object_2', 'preempted':'Give_Object_2'})
            smach.StateMachine.add(
                'Say_Rescue_stay',
                text_to_say('Please Stay here I am going to call for the Ambulance'),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'aborted':'preempted'})



