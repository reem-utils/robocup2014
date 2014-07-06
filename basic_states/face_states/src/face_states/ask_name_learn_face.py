#! /usr/bin/env python
'''
Created on 08/03/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
from face_states.new_database_and_learn import new_database_and_learn
from speech_states.say import text_to_say
from speech_states.ask_question import AskQuestionSM
from speech_states.parser_grammar import parserGrammar

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
GRAMMAR_NAME = "robocup/iam"

class prepare_name(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['asr_answer', 'asr_answer_tags'],
                                output_keys=['name'])
        self.tags = parserGrammar(GRAMMAR_NAME)

    def execute(self, userdata):
        for element in self.tags:
            if element[0] == 'nameshort' or element[0] == 'nameall':
                for value in element[1]:
                    if value in userdata.asr_answer:
                        userdata.name = value
                        return 'succeeded'
        return 'aborted'
    
class prepare_say_name(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['name'],
                                output_keys=['tts_text','tts_wait_before_speaking', 'tts_lang'])

    def execute(self, userdata):

        userdata.tts_text = "Nice to meet you " + userdata.name
        userdata.tts_wait_before_speaking = 0
        userdata.tts_lang = None
        
        return 'succeeded'
    
class SaveFaceSM(smach.StateMachine):
    """
    Executes a SM that learns one face. 
    The robot listen the name from the person and then learns his face. 

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
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                        input_keys=[],
                                        output_keys=['name'])

        with self:
            self.userdata.name = ''
            self.userdata.asr_userSaid = ''
            self.userdata.grammar_name = ''
            
            # Ask for name
            smach.StateMachine.add(
                'listen_name',
                AskQuestionSM("Hi, what's your name?", GRAMMAR_NAME),
                transitions={'succeeded': 'prepare_name', 'aborted': 'ask_name_again', 
                'preempted': 'preempted'}) 
            
            # We prepare the name for face_detection 
            smach.StateMachine.add(
                "prepare_name",
                prepare_name(),
                transitions={'succeeded': 'say_start', 'aborted': 'ask_name_again', 
                'preempted': 'preempted'})  
            
            # Ask for name again
            smach.StateMachine.add(
                'ask_name_again',
                AskQuestionSM("Sorry, I don't understand you. Can you repeat your name, please?", GRAMMAR_NAME),
                transitions={'succeeded': 'prepare_name', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
           
            smach.StateMachine.add(
                'say_start',
                text_to_say("OK,  now i am going to enroll your  face, don't  move"),
                transitions={'succeeded': 'learn_face', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            

            # Start learning
            smach.StateMachine.add(
                'learn_face',
                 new_database_and_learn(10),
                transitions={'succeeded': 'prepare_say_name', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Say learn name
            smach.StateMachine.add(
                'prepare_say_name',
                prepare_say_name(),
                transitions={'succeeded': 'say_name', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'say_name',
                text_to_say(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
           

