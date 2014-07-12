'''
Created on 10/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy

from speech_states.say import text_to_say
from gesture_states.wave_detection_sm import WaveDetection
from navigation_states.nav_to_coord import nav_to_coord
from face_states.detect_faces import detect_face
from face_states.ask_name_learn_face import SaveFaceSM
from speech_states.ask_question import AskQuestionSM

GRAMMAR_NAME = "robocup/drinks"

class prepare_coord_wave(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['wave_position', 'wave_yaw_degree', 'nav_to_coord_goal'],
                                output_keys=['standard_error', 'nav_to_coord_goal'])
    def execute(self, userdata):
        
        x = userdata.wave_position.point.x
        y = userdata.wave_position.point.y
        yaw  = userdata.wave_yaw_degree
        
        userdata.nav_to_coord_goal = [x, y, yaw]
        rospy.logwarn("X: " + str(x) + " Y: " + str(y) + " yaw: " + str(yaw))
        
        return 'succeeded' 

class process_order(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=["asr_answer","asr_answer_tags"],
                                output_keys=['object_name'])
        
    def execute(self, userdata):
    
        tags = [tag for tag in userdata.asr_answer_tags if tag.key == 'object']
        if tags:
            userdata.object_name = tags[0].value
            return 'succeeded'
         
        return 'aborted'
    
class AskOrder(smach.StateMachine):
    """
    Executes a SM that ask for a order in cocktail Party.
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=[],
                                    output_keys=['name', 'object_name'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!

            # Say Wave recognize
            smach.StateMachine.add(
                 'say_search_wave',
                 text_to_say("I'm searching for people waving at me", wait=False),
                 transitions={'succeeded': 'wave_recognition', 'aborted': 'wave_recognition'}) 
            
            # Gesture recognition -> Is anyone waving?
            smach.StateMachine.add(
                'wave_recognition',
                WaveDetection(time_for_wave=20),
                transitions={'succeeded': 'say_wave_recognize', 'aborted': 'ask_for_person', 
                'preempted': 'preempted'}) 
            
            # Say Wave recognize
            smach.StateMachine.add(
                 'say_wave_recognize',
                 text_to_say("Someone waved to me. I will go there", wait=False),
                 transitions={'succeeded': 'prepare_coord_wave', 'aborted': 'prepare_coord_wave'}) 
           
        # Person Recognize   
            # Prepare the goal to the person that is waving
            # TODO: it goes a little far to the person... 
            smach.StateMachine.add(
                'prepare_coord_wave',
                prepare_coord_wave(),
                transitions={'succeeded': 'go_to_person_wave', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Go to the person -> we assume that gesture will return the position
            smach.StateMachine.add(
                'go_to_person_wave',
                nav_to_coord('/base_link'),
                transitions={'succeeded': 'learning_person', 'aborted': 'go_to_person_wave', 
                'preempted': 'preempted'}) 
        
        # FAIL Person Recognize
            # Ask for person if it can see anyone
            smach.StateMachine.add(
                'ask_for_person',
                text_to_say("I can't see anyone. Can anyone come to me, please?"),
                transitions={'succeeded': 'wait_for_person', 'aborted': 'ask_for_person', 
                'preempted': 'preempted'}) 
            
            # Wait for person
            smach.StateMachine.add(
                 'wait_for_person',
                 detect_face(),
                 transitions={'succeeded': 'learning_person', 'aborted': 'ask_for_person'})
            
            # Learn Person -> Ask name + Face Recognition
            smach.StateMachine.add(
                'learning_person',
                SaveFaceSM(),
                transitions={'succeeded': 'ask_order', 'aborted': 'learning_person', 
                'preempted': 'preempted'}) 
            
            # Ask for order
            smach.StateMachine.add(
                'ask_order',
                AskQuestionSM("What would you like to order?", GRAMMAR_NAME),
                transitions={'succeeded': 'process_order', 'aborted': 'ask_order', 
                'preempted': 'preempted'}) 

            # Process the answer
            smach.StateMachine.add(
                'process_order',
                process_order(),
                transitions={'succeeded': 'say_got_it', 'aborted': 'ask_order', 
                'preempted': 'preempted'}) 
        
            # Say what he ask
            smach.StateMachine.add(
                'say_got_it',
                text_to_say("I got it!"),
                transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 
                'preempted': 'preempted'}) 
