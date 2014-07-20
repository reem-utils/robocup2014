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

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class dummy_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['object_position','pose_to_place','nav_to_poi_name'], 
            output_keys=['object_position','pose_to_place', 'nav_to_poi_name'])

    def execute(self, userdata):
        
        userdata.object_position = PoseStamped()
        userdata.object_position.header.frame_id = "base_link"
        userdata.object_position.pose.position.x = 0.5
        userdata.object_position.pose.position.z = 1.0
        userdata.object_position.pose.orientation.w = 1.0
        userdata.pose_to_place = PoseStamped()
        userdata.pose_to_place.header.frame_id = "base_link"
        userdata.pose_to_place.pose.position.x = 0.4
        userdata.pose_to_place.pose.position.z = 0.95
        userdata.pose_to_place.pose.orientation.w = 1.0
        userdata.nav_to_poi_name='dinner_table'
         
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
            self.userdata.nav_to_poi_name=''
            
#             # Do the Macarena Dance
#             smach.StateMachine.add(
#                 'do_macarena',
#                 #DummyStateMachine(),
#                 play_motion_sm(motion='macarena'),
#                 transitions={'succeeded': 'do_YMCA', 'aborted': 'aborted', 
#                 'preempted': 'preempted'}) 
# 
#             # Do the YMCA Dance
#             smach.StateMachine.add(
#                 'do_YMCA',
#                 play_motion_sm(motion='ymca'),
#                 transitions={'succeeded': 'do_robot', 'aborted': 'aborted', 
#                 'preempted': 'preempted'})    
# 
#             # Do the Robot Dance
#             smach.StateMachine.add(
#                 'do_robot',
#                 play_motion_sm('robot'),
#                 transitions={'succeeded': 'succeed', 'aborted': 'aborted', 
#                 'preempted': 'preempted'}) 

            # Do object_recognition 
            smach.StateMachine.add(
                'object_recognition',
                dummy_recognize(),
                transitions={'succeeded': 'say_grasp_object', 'aborted': 'say_release_obj', 
                'preempted': 'preempted'}) 
            
            # Say grasp object
            smach.StateMachine.add(
                 'say_grasp_object',
                 text_to_say("I'm going to grasp the object"),
                 transitions={'succeeded': 'grasp_object', 'aborted': 'grasp_object'})
            
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                pick_object_sm(),
                transitions={'succeeded': 'release_object', 'aborted': 'play_motion_state',
                'preempted': 'preempted'}) 
            
            # Release the object
            smach.StateMachine.add(
                'release_object',
                place_object_sm(),
                transitions={'succeeded': 'play_motion_state', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Home position
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home'),
                transitions={'succeeded': 'say_grasp_object',
                             'preempted':'say_grasp_object', 
                             'aborted':'say_grasp_object'})