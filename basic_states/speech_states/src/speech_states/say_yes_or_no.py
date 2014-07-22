#! /usr/bin/env python
'''
Created on 28/03/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
from speech_states.say import text_to_say
from speech_states.listen_to import ListenToSM
from speech_states.parser_grammar import parserGrammar
from read_asr import ReadASR
from pal_interaction_msgs.msg._ASREvent import ASREvent
from speech_states.activate_asr import ActivateASR

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
    
GRAMMAR_NAME = 'robocup/yes_no'

class ProcessCommand(smach.State):
    def __init__(self):
        rospy.loginfo("Entering SelectAnswer")
        smach.State.__init__(self, outcomes=['yes', 'no', 'aborted'], 
                                input_keys=['asr_userSaid', 'asr_userSaid_tags'],
                                output_keys=['standard_error'])
      #  self.tags = parserGrammar(GRAMMAR_NAME)
        
    def execute(self, userdata):        
        question = userdata.asr_userSaid
        questionTags = userdata.asr_userSaid_tags

        # We need to compare the userSaid with all the possible values that we can recognize
        
        yes = [tag for tag in questionTags if tag.key == 'yes']
        no = [tag for tag in questionTags if tag.key == 'no']
          
        print yes
        print no
          
        if yes:
            rospy.loginfo("YES")
            return 'yes'
        if no: 
            rospy.loginfo("NO")
            return 'no'
          
        rospy.loginfo("Aborted")
        return 'aborted'
        
#         for element in self.tags:
#             for value in element[1]:
#                 if userdata.asr_userSaid == value:
#                     return element[0]
#          
#         return 'aborted'
     
class SayYesOrNoSM(smach.StateMachine):
    """
    Executes a SM that ask for confirmation. 
    The robot listen the person and returns: 
    -succeeded   in case that he said yes
    -aborted     if said no. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """    
    def __init__(self,bucle=True,grammar=None,calibrate=False,Time_calibrate=12):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                        input_keys=[],
                                        output_keys=[])

        with self:
            self.userdata.asr_userSaid = ''    
            self.userdata.grammar_name = GRAMMAR_NAME
            self.userdata.tts_text = None
            self.userdata.tts_wait_before_speaking = None
            self.userdata.tts_lang = None
            
            # Listen 
            smach.StateMachine.add(
                'listen_info',
                ReadASR(bucle=bucle,calibrate=calibrate,Time_calibrate=Time_calibrate),
                transitions={'succeeded': 'process_command', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Prepare the information 
            smach.StateMachine.add(
                "process_command",
                ProcessCommand(),
                transitions={'yes': 'succeeded', 'no': 'aborted', 'aborted':'repeat'})  

            # Ask for repeat
            smach.StateMachine.add(
                'repeat',
                text_to_say("Excuse me, I don't understand you. Can you repeat?"),
                transitions={'succeeded': 'listen_info', 'aborted': 'aborted', 
                'preempted': 'preempted'})             
           

class SayYesOrNoSM_2(smach.StateMachine):
    """
    Executes a SM that ask for confirmation. 
    The robot listen the person and returns: 
    -succeeded   in case that he said yes
    -aborted     if said no. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """    
    def __init__(self, grammar=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                        input_keys=[],
                                        output_keys=[])

        with self:
            self.userdata.asr_userSaid = ''    
            self.userdata.grammar_name = GRAMMAR_NAME
            self.userdata.tts_text = None
            self.userdata.tts_wait_before_speaking = None
            self.userdata.tts_lang = None
            
            
            
            # Activate the server
            smach.StateMachine.add('ActivateASR',
                    ActivateASR(GRAMMAR_NAME),
                    transitions={'succeeded': 'listen_info', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Listen 
            smach.StateMachine.add(
                'listen_info',
                ReadASR(),
                transitions={'succeeded': 'process_command', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Prepare the information 
            smach.StateMachine.add(
                "process_command",
                ProcessCommand(),
                transitions={'yes': 'succeeded', 'no': 'aborted', 'aborted':'aborted'})  

            # Ask for repeat
            smach.StateMachine.add(
                'repeat',
                text_to_say("Excuse me, I don't understand you. Can you repeat?"),
                transitions={'succeeded': 'aborted', 'aborted': 'aborted', 
                'preempted': 'preempted'})             
           