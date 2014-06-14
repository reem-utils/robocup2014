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
from navigation_states.nav_to_coord_ud import nav_to_coord_ud
from speech_states.say import text_to_say
from search_faces import SearchFacesSM
from navigation_states.get_current_robot_pose import get_current_robot_pose
from util_states.math_utils import add_vectors, substract_vector
from util_states.pose_at_distance import pose_at_distance, pose_at_distance2
from geometry_msgs.msg import Pose
from speech_states.parser_grammar import parserGrammar
from util_states.math_utils import *
from face_states.detect_faces import detect_face
from speech_states.activate_asr import ActivateASR
from speech_states.deactivate_asr import DeactivateASR
from speech_states.read_asr import ReadASR
from manipulation_states.play_motion_sm import play_motion_sm
from smach.user_data import Remapper
from tf.transformations import quaternion_from_euler
# Constants
NUMBER_OF_QUESTIONS = 3
GRAMMAR_NAME = 'robocup/what_did_you_say_2'

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class prepare_coord_person(smach.State):
    def __init__(self, distanceToHuman=0.3):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['face', 'nav_to_coord_goal', 'face_frame','current_robot_yaw'],
                                output_keys=['standard_error', 'nav_to_coord_goal', 'nav_to_coord_frame'])
        self.distanceToHuman = distanceToHuman
        
    def execute(self, userdata):
        
        new_pose = Pose()
        # TODO: We adapt the coordinates respect Kinect 
        new_pose.position.x = userdata.face.position.z
        new_pose.position.y = -userdata.face.position.x
        
        unit_vector = normalize_vector(new_pose.position)
        position_distance = vector_magnitude(new_pose.position)

        distance_des = 0.0
        if position_distance >= self.distanceToHuman: 
            distance_des = position_distance - self.distanceToHuman
        else:
            rospy.loginfo(" Person too close => not moving, just rotate")
 
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        
        #userdata.nav_to_coord_goal = [new_pose.position.x, new_pose.position.y, alfa]
        #userdata.nav_to_coord_goal = [new_pose.position.x, new_pose.position.y, userdata.current_robot_yaw]
        
        userdata.nav_to_coord_goal = [userdata.face.position.x,userdata.face.position.y, userdata.face.position.z-0.4, math.radians(0),math.radians(0), math.radians(-90)]
        rospy.loginfo('FACE FRAME PREPARE COORD: ' + str(userdata.face_frame))
        userdata.nav_to_coord_frame = userdata.face_frame
        return 'succeeded'

class prepear_repeat(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['asr_userSaid'], output_keys=['tts_text'])
    def execute(self, userdata):

        userdata.tts_text = "You said " + userdata.asr_userSaid 
         
        return 'succeeded'

class SelectAnswer(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'None'], 
                                input_keys=['asr_userSaid', 'asr_userSaid_tags'],
                                output_keys=['standard_error', 'tts_text', 'tts_wait_before_speaking'])
        #self.tags = parserGrammar(GRAMMAR_NAME)
        
    def execute(self, userdata):        
        question = userdata.asr_userSaid
        questionTags = userdata.asr_userSaid_tags
        question_params = rospy.get_param("/question_list/questions/what_say_simple")
        question_number = [tag for tag in questionTags if tag.key == 'questionumber']
        rospy.loginfo("Question TAGS :: " + str(questionTags))
        rospy.loginfo("Question TAGS NUMBER:: " + str(question_number))
        rospy.loginfo("Question TAGS NUMBER 0 :: " + str(question_number[0]))
        
        for key,value in question_params.iteritems():
            print "Key: " + str(key)
            print "Value: " + str(value)
            
            if str(question_number[0].value) == str(value[2]):
                print "FOUND!"
                userdata.tts_text = "The answer is " + str(value[3])
                userdata.tts_wait_before_speaking = 0
                userdata.standard_error=''
                return 'succeeded'
            
            # Process info -> value[2] and Process country -> value[3]
#             if value[2] in userdata.asr_userSaid and value[3] in userdata.asr_userSaid:
#                 # Select answer -> value[4]
#                 userdata.tts_text = "The answer is " + value[4]
#                 userdata.tts_wait_before_speaking = 0
#                 userdata.standard_error=''
#                 return 'succeeded'
                
        userdata.tts_text = "I don't know the answer"
        userdata.tts_wait_before_speaking = 0
        userdata.standard_error='Answer not found'
        rospy.logerr('ANSWER NOT FOUND')    
        return 'aborted'
        
        
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
            self.userdata.nav_to_poi_name = ""
            self.userdata.faces = ""
            self.userdata.name=''
            self.userdata.tts_text = None
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_lang = ''
            
            # Listen the first question
            self.userdata.grammar_name = GRAMMAR_NAME
            

                     
            # Enter room
            smach.StateMachine.add(
                 'say_what_did_you_say',
                 text_to_say("I'm beginning the what did you say test,. I'm going to the place where the referee should be"),
                 #transitions={'succeeded': 'go_location', 'aborted': 'aborted'})
                 transitions={'succeeded': 'go_location', 'aborted': 'aborted'})
            
            # Go to the location
            smach.StateMachine.add(
                 'go_location',
                 nav_to_poi("find_me"),
                 transitions={'succeeded': 'search_face', 'aborted': 'aborted', 
                 'preempted': 'preempted'})    
             
#             smach.StateMachine.add(
#                  'say_faces',
#                  text_to_say("Searching for faces"),
#                  transitions={'succeeded': 'search_face', 'aborted': 'aborted'})
            
            # Look for a face
            smach.StateMachine.add(
                 'search_face',
                 SearchFacesSM(),
                 transitions={'succeeded': 'Say_Found_Face', 'aborted': 'ask_for_tc', 
                 'preempted': 'preempted'},
                  remapping={'face_frame':'face_frame'})
             
            # Go to the person - We assume that find person will return the position for the person
            smach.StateMachine.add(
                                   'Say_Found_Face',
                                   text_to_say('Referee! I have found you at last. Now I am going to you, wait for me.'),
                                   transitions={'succeeded': 'get_current_pose_yaw', 'aborted': 'Say_Found_Face', 
                                                'preempted': 'preempted'})
            smach.StateMachine.add(
                                   'get_current_pose_yaw',
                                   get_current_robot_pose(),
                                   transitions={'succeeded': 'prepare_coord_person', 'aborted': 'ask_for_tc',
                                                    'preempted': 'preempted'})
            smach.StateMachine.add(
                 'prepare_coord_person',
                 prepare_coord_person(),
                 transitions={'succeeded': 'go_to_person', 'aborted': 'aborted',
                 'preempted': 'preempted'})
            
            
            smach.StateMachine.add(
                 'go_to_person',
                 nav_to_coord_ud(),
                 transitions={'succeeded': 'say_found', 'aborted': 'Say_person_not_reached',
                 'preempted': 'preempted'})
            smach.StateMachine.add(
                                   'Say_person_not_reached',
                                   text_to_say('I Found you, but cannot reach you, can you come to me please?'),
                                   transitions={'succeeded': 'ActivateASR', 'aborted': 'aborted'})
             
            # Say "I found you!" + Small Talk
            smach.StateMachine.add(
                 'say_found',
                 text_to_say("I found you! I have arrived to where you are."),
                 transitions={'succeeded': 'ActivateASR', 'aborted': 'aborted'})
             
            # Ask for TC if we don't find him
            smach.StateMachine.add(
                 'ask_for_tc',
                 text_to_say("I can't find you. Can you come to me?"),
                 transitions={'succeeded': 'wait_for_tc', 'aborted': 'aborted'})
              
            # Wait for TC
            smach.StateMachine.add(
                 'wait_for_tc',
                 detect_face(),
                 transitions={'succeeded': 'say_found', 'aborted': 'aborted'})
            
            # Question Part ---------------------------------------------------------------------------------------------
            
            # Init the asr service

            # Activate the server
            smach.StateMachine.add('ActivateASR',
                    ActivateASR(),
                    transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # loop test - It checks the number of iterations
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'ask_next_question', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'DeactivateASR'})
            
            # Ask for next question
            smach.StateMachine.add(
                'ask_next_question',
                text_to_say("I'm ready, ask me a question"),
                transitions={'succeeded': 'listen_question', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'listen_question',
                ReadASR(),
                transitions={'succeeded': 'prepear_repeat', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            smach.StateMachine.add(
                'prepear_repeat',
                prepear_repeat(),
                transitions={'succeeded': 'repeat_question', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
             
            # Repeat question
            smach.StateMachine.add(
                'repeat_question',
                text_to_say(),
                transitions={'succeeded': 'search_answer', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Search the answer
            smach.StateMachine.add(
                'search_answer',
                SelectAnswer(),
                transitions={'succeeded': 'say_answer', 'aborted': 'say_answer', 
                'preempted': 'preempted', 'None': 'aborted'})    

            # Say the answer
            smach.StateMachine.add(
                'say_answer',
                text_to_say(),
                transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Deactivate ASR
            smach.StateMachine.add('DeactivateASR',
                DeactivateASR(),
                transitions={'succeeded': 'say_end', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                 'say_end',
                 text_to_say("What did you say test finished"),
                 transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
def main():
    rospy.loginfo('what say Node')
    rospy.init_node('what_say_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'what_say',
            WhatSaySM(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()
