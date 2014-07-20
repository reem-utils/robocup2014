#! /usr/bin/env python
'''
Created on 10/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy
import sys

import smach_ros
import sys
import actionlib

from speech_states.say import text_to_say
from gesture_states.wave_detection_sm import WaveDetection
from navigation_states.nav_to_coord import nav_to_coord
from face_states.detect_faces import detect_face
from face_states.ask_name_learn_face import SaveFaceSM
from speech_states.ask_question import AskQuestionSM
from speech_states.asr_calibrate import CalibrateASR

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
    
class listen_repeat_calibrate(smach.StateMachine):
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
    def __init__(self,GRAMMAR_NAME):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['delete_database'],
                                    output_keys=['name_face', 'object_name'])
        self.grammar_name=GRAMMAR_NAME
        with self:
        
         
            # Ask for order
            smach.StateMachine.add(
                'ask_order',
                AskQuestionSM("tell me", self.grammar_name),
                transitions={'succeeded': 'succeeded', 'aborted': 'calibrate', 
                'preempted': 'preempted'}) 

            # Process the answer
            smach.StateMachine.add(
                'process_order',
                process_order(),
                transitions={'succeeded': 'say_got_it', 'aborted': 'ask_order',  # TODO before it was ask_order
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'calibrate',
                CalibrateASR(),
                transitions={'succeeded': 'ask_order', 'aborted': 'ask_order', 
                'preempted': 'preempted'})
            
        
            # Say what he ask
            smach.StateMachine.add(
                'say_got_it',
                text_to_say("I got it!"),
                transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 
                'preempted': 'preempted'}) 
            
            
            
def main():
    rospy.init_node('listen_repeat_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    print "put the name of the gramar"
    lineRead = sys.stdin.readline()
    GRAMMAR_NAME = "robocup/"+lineRead[:len(lineRead)-1]
    with sm:
        sm.userdata.grammar_name = None

        smach.StateMachine.add('ListenRepeatTest',
            listen_repeat_calibrate(GRAMMAR_NAME),
            transitions={'succeeded': 'ListenRepeatTest', 'aborted': 'ListenRepeatTest'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'listen_repeat_test_introspection', sm, '/LISTEN_REPEAT_TEST')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()