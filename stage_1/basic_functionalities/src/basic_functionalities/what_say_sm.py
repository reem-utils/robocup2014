#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Sergi Xavier Ubach Pallàs
@email: sxubach@gmail.com

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

26 Feb 2014
"""

import rospy
import smach
from speech_states.listen_to import ListenToSM
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.nav_to_coord import nav_to_coord
from speech_states.say import text_to_say
from search_faces import SearchFacesSM

from navigation_states.get_current_robot_pose import get_current_robot_pose
from util_states.math_utils import add_vectors, substract_vector
from util_states.pose_at_distance import pose_at_distance, pose_at_distance2
from geometry_msgs.msg import Pose



# Constants
NUMBER_OF_QUESTIONS = 3
GRAMMAR_NAME = 'what_did_you_say.gram'

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class prepare_coord_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['face', 'current_robot_yaw','current_robot_pose'], 
            output_keys=['nav_to_coord_goal', 'standard_error'])

    def execute(self,userdata):
        person_pose = Pose()
        person_pose.position = userdata.face.position
        person_pose.orientation = userdata.current_robot_pose.pose.orientation
        
        person_pose = pose_at_distance2(userdata.current_robot_pose.pose,person_pose,1.41)
        userdata.nav_to_coord_goal=[person_pose.position.x, person_pose.position.y, userdata.current_robot_yaw]
    
        return 'succeeded'

class SelectAnswer(smach.State):
    def __init__(self):
        rospy.loginfo("Entering SelectAnswer")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'None'], 
                                input_keys=['asr_userSaid', 'asr_userSaid_tags'],
                                output_keys=['standard_error', 'tts_text', 'tts_wait_before_speaking'])

    def execute(self, userdata):        
        question = userdata.asr_userSaid
        questionTags = userdata.asr_userSaid_tags
        foundAnswer = False
        #important to do add the .yalm before
        question_params = rospy.get_param("/question_list/questions/what_say")
       
        info = [tag for tag in questionTags if tag.key == 'info']
        country = [tag for tag in questionTags if tag.key == 'country']
        
        for key, value in question_params.iteritems():
            
            if (info and info[0].value == value[2]) and (country and country[0].value == value[3]):
                userdata.tts_text = value[4]
                userdata.tts_wait_before_speaking = 0
                foundAnswer = True
                break

        if foundAnswer:
            userdata.standard_error='OK'
            return 'succeeded'
   
        else:            
            userdata.tts_text = "I don't know"
            userdata.tts_wait_before_speaking = 0
            userdata.standard_error='Answer not found'
            rospy.logerr('ANSWER NOT FOUND')
            return 'succeeded'
        
        
class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['loop_iterations'],
                                output_keys=['standard_error', 'loop_iterations'])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_QUESTIONS:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'

    
def child_term_cb(outcome_map):

    #If time passed, we terminate all running states
    if outcome_map['TimeOut'] == 'succeeded':
        rospy.loginfo('TimeOut finished')
        return True
    
    if outcome_map['search_face'] == 'succeeded':
        rospy.loginfo('search_face finished')
        return True
    #By default, just keep running
    return False


def out_cb(outcome_map):
    if outcome_map['TimeOut'] == 'succeeded':
        return 'succeeded'   
    elif outcome_map['search_face'] == 'succeeded':
        return 'succeeded'   
    else:
        return 'aborted'


class WhatSaySM(smach.StateMachine):
    """
    Executes a SM that does the test to what did you say.
    In this test the robot goes into a room with a person and it must find it. 
    When it found her, it announced that it found the person and starts a small talk.
    If after a minute the robot don't find the person it ask to walk in from on it. 
    
    In this point the robot will be asked 3 questions, it repeats the question and give an answer.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    To use this SM in a simulator is required to run asr_srv.py, tts_as and roscore.
    """
    
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 0
            self.userdata.name=''
            
            # Listen the first question
            self.userdata.grammar_name = GRAMMAR_NAME
            
            # Find me Part 
            
            # Enter room

            # Go to the location
            smach.StateMachine.add(
                'go_location',
                nav_to_poi("find_me"),
                transitions={'succeeded': 'search_face', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
            
            # Look for a face
            smach.StateMachine.add(
                'search_face',
                SearchFacesSM(),
                transitions={'succeeded': 'get_actual_pos', 'aborted': 'ask_for_tc', 
                'preempted': 'preempted'})
            
            # Where are we?
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pose(),
                transitions={'succeeded': 'prepare_coord_person', 'aborted': 'aborted', 'preempted': 'succeeded'})
            
            # Go to the person - We assume that find person will return the position for the person
            smach.StateMachine.add(
                'prepare_coord_person',
                prepare_coord_person(),
                transitions={'succeeded': 'go_to_person', 'aborted': 'aborted', 
                'preempted': 'preempted'})                    
            
            smach.StateMachine.add(
                'go_to_person',
                nav_to_coord(),
                transitions={'succeeded': 'say_found', 'aborted': 'aborted', 
                'preempted': 'preempted'})   
            
            # Say "I found you!" + Small Talk
            smach.StateMachine.add(
                'say_found',
                text_to_say("I found you!"),
                transitions={'succeeded': 'loop_test', 'aborted': 'aborted'})
            
            # Ask for TC if we dont find him
            smach.StateMachine.add(
                'ask_for_tc',
                text_to_say("I can't find you. Can you come to me?"),
                transitions={'succeeded': 'loop_test', 'aborted': 'aborted'})
            
            # Question Part ---------------------------------------------------------------------------------------------
            
            # loop test - It checks the number of iterations
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'ask_next_question', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'succeeded'})
            
            # Ask for next question
            smach.StateMachine.add(
                'ask_next_question',
                text_to_say("I'm ready, ask me a question"),
                transitions={'succeeded': 'listen_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'listen_question',
                ListenToSM(),
                transitions={'succeeded': 'search_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  

            # Search the answer
            smach.StateMachine.add(
                'search_answer',
                SelectAnswer(),
                transitions={'succeeded': 'say_answer', 'aborted': 'aborted', 
                'preempted': 'preempted', 'None': 'aborted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_answer',
                text_to_say(),
                transitions={'succeeded': 'loop_test', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            