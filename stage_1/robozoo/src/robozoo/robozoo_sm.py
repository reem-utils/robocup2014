#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author:  Cristina De Saint Germain
@email: crsaintc8@gmail.com

@author:  Sergi Xavier Ubach Pallàs
@email: sxubach@gmail.com

12 Mar 2014
"""

import rospy
import smach

from manipulation_states.play_motion_sm import play_motion_sm
from object_grasping_states.pick_object_sm import pick_object_sm
from object_grasping_states.place_object_sm import place_object_sm
from speech_states.say import text_to_say
from geometry_msgs.msg import PoseStamped
from util_states.state_concurrence import ConcurrenceRobocup
from util_states.sleeper import Sleeper

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class dummy_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['object_position','pose_to_place'], 
            output_keys=['object_position','pose_to_place'])

    def execute(self, userdata):
        
        userdata.object_position = PoseStamped()
        userdata.object_position.header.frame_id = "base_link"
        userdata.object_position.pose.position.x = 0.95
        userdata.object_position.pose.position.z = 1.00
        userdata.object_position.pose.orientation.w = 1.0
        userdata.pose_to_place = PoseStamped()
        userdata.pose_to_place.header.frame_id = "base_link"
        userdata.pose_to_place.pose.position.x = 0.95
        userdata.pose_to_place.pose.position.z = 1.00
        userdata.pose_to_place.pose.orientation.w = 1.0
         
        rospy.sleep(5)
        return 'succeeded'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class RoboZooSM(smach.StateMachine):
    """
    Executes a SM that does the robozoo
    The robot dances for an hour. It does different dances in a loop. 

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

            # Robot presentation: Little talk + wave gesture
            STATES = [text_to_say("Hi everybody! My name is REEM."), play_motion_sm("wave")]
            STATE_NAMES = ["say_presentation", "salute_wave"]
            outcome_map = {'succeeded': {"say_presentation": 'succeeded', "salute_wave": 'succeeded'}}
        
            smach.StateMachine.add(
                "robot_presentation",
                ConcurrenceRobocup(states=STATES, state_names=STATE_NAMES, outcome_map=outcome_map),
                transitions={'succeeded': 'object_recognition', 'aborted': "robot_presentation"})
            
            # Do object_recognition 
            smach.StateMachine.add(
                'object_recognition',
                dummy_recognize(),
                transitions={'succeeded': 'say_grasp_object', 'aborted': 'say_grasp_object', 
                'preempted': 'preempted'}) 
            
            # Say grasp object
            smach.StateMachine.add(
                 'say_grasp_object',
                 text_to_say("I am hungry. I am going to pick the noodles"),
                 transitions={'succeeded': 'grasp_object', 'aborted': 'grasp_object'})
            
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                pick_object_sm(),
                transitions={'succeeded': 'say_why_object', 'aborted': 'play_motion_state',
                'preempted': 'preempted'}) 
            
            # Say release object
            smach.StateMachine.add(
                 'say_why_object',
                 text_to_say("I am a robot, I can't eat this. I am so sad! I am going to leave the noodles in the table", wait=False),
                 transitions={'succeeded': 'release_object', 'aborted': 'release_object'})
            
            # Release the object
            smach.StateMachine.add(
                'release_object',
                place_object_sm(),
                transitions={'succeeded': 'robot_finish', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Say hard job + bow
            STATES = [text_to_say("Uff, this was a hard job. Thank you very much for your attention"), play_motion_sm("bow")]
            STATE_NAMES = ["say_hard_job", "motion_bow"]
            outcome_map = {'succeeded': {"say_hard_job": 'succeeded', "motion_bow": 'succeeded'}}
        
            smach.StateMachine.add(
                "robot_finish",
                ConcurrenceRobocup(states=STATES, state_names=STATE_NAMES, outcome_map=outcome_map),
                transitions={'succeeded': 'say_rest', 'aborted': "say_rest"})
       
            # Say go rest
            smach.StateMachine.add(
                 'say_rest',
                 text_to_say("I am going to rest for a few seconds", wait = False),
                 transitions={'succeeded': 'sleep_robot', 'aborted': 'sleep_robot'})
 
            # Sleep
            smach.StateMachine.add(
                'sleep_robot',
                Sleeper(30),
                transitions={'succeeded': 'robot_presentation',
                             'preempted':'robot_presentation', 
                             'aborted':'robot_presentation'})
            
            # Home position
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home'),
                transitions={'succeeded': 'say_grasp_object',
                             'preempted':'say_grasp_object', 
                             'aborted':'say_grasp_object'})