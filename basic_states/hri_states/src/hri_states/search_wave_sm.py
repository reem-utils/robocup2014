#! /usr/bin/env python
"""
Created on 28/05/14

@author: Chang Long Zhu Jin
@mail: changlongzj@gmail.com
"""

import rospy
import smach
import tf
import tf.transformations as TT
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point, Quaternion
from util_states.topic_reader import topic_reader
from pal_vision_msgs.msg import Gesture
import numpy
import copy
from speech_states.say import text_to_say
from util_states.math_utils import *
from gesture_states.wave_detection_sm import WaveDetection
from manipulation_states.move_head_form import move_head_form

GESTURE_TOPIC = '/head_mount_xtion/gestures'
final_frame_id = 'base_link'
PUB_1_TOPIC = '/given_pose'
PUB_2_TOPIC = '/transformed_pose'

NUMBER_OF_HEAD_POSES = 3

class prepare_move_head(smach.State):
    def __init__(self, head_position):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end_searching'], 
                                input_keys=['loop_iterations', 'head_left_right', 'head_up_down'],
                                output_keys=['head_left_right', 'head_up_down', 'standard_error', 'loop_iterations'])
        self.head_position = head_position

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_HEAD_POSES:
            userdata.loop_iterations = 0
            return 'end_searching'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            
            userdata.head_left_right = 1 - userdata.loop_iterations*0.5
            
            if userdata.loop_iterations == 0:
                userdata.head_left_right = 'mid_left'
            elif userdata.loop_iterations == 1:
                userdata.head_left_right = 'center'
            elif userdata.loop_iterations == 2:
                userdata.head_left_right = 'mid_right'

            userdata.head_up_down = self.head_position if self.head_position else 'normal'

            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'

class Search_Wave_SM(smach.StateMachine):
    """
        Search_Wave_SM: This SM searches for any 'wave' gesture around a room, 
                        as the robot will move its head from left to right.
                        If any gesture is detected, then it will stop and register the position XYZ of the gesture.

        Input Keys:
            None
        Output Keys:
            @key wave_position: A PointStamped point referenced to /base_link
            @key wave_yaw_degree: the yaw in degrees for the robot to turn.
            @key standard_error: A base error to inform.

        Required Parameters: 
            None
            
        Outcomes:
            'succeeded' : Found a person
            'aborted' : something went wrong
            'end_searching' : No one is found, so searching is cancelled

    """
    def __init__(self, head_position=None, text_for_wave_searching="I am looking for you."):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'end_searching'],
                                 input_keys=[],
                                 output_keys=['wave_position', 'wave_yaw_degree','standard_error'])
        with self:
            self.userdata.loop_iterations = 0
            self.userdata.wave_position = None
            self.userdata.wave_yaw_degree = None
            self.userdata.standard_error = ''
            smach.StateMachine.add(
                                   'Move_head_prepare',
                                   prepare_move_head(head_position),
                                    transitions={'succeeded': 'move_head', 'aborted': 'aborted', 
                                                'preempted': 'preempted', 'end_searching':'end_searching'})
            smach.StateMachine.add(
                                   'move_head',
                                   move_head_form(head_up_down=head_position),
                                   transitions={'succeeded': 'Say_Searching',
                                                'preempted':'Say_Searching',
                                                'aborted':'aborted'})
            smach.StateMachine.add(
                'Say_Searching',
                text_to_say(text_for_wave_searching),
                transitions={'succeeded':'wave_recognition', 'aborted':'wave_recognition', 'preempted':'wave_recognition'})
           
            #Hard-coded maximum time in order to detect wave 
            smach.StateMachine.add(
                'wave_recognition',
                WaveDetection(4.0),
                transitions={'succeeded': 'Say_Found', 'aborted': 'Move_head_prepare', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'Say_Found',
                text_to_say("Oh! I have found you finally."),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
            




def main():
    rospy.loginfo('Wave Detection Node')
    rospy.init_node('wave_detection_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'end_searching'])
    with sm:
        smach.StateMachine.add(
            'gesture_state',
            Search_Wave_SM(head_position='down'),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted', 'end_searching':'end_searching'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()








