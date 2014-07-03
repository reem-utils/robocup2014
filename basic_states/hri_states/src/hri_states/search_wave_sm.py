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

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

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

# gets called when ANY child state terminates
def child_term_cb(outcome_map):
   
    if outcome_map['wave_recognition']:
        rospy.loginfo(OKGREEN + "wave_recognition ends" + ENDC)
        return True
    
#     if outcome_map['move_head'] == 'succeeded':
#         rospy.loginfo(OKGREEN + "move_head ends" + ENDC)
#         
#         return False
# 
#     # terminate all running states if BAR finished
#     if outcome_map['Say_Searching'] == 'succeeded':
#         rospy.loginfo(OKGREEN + "Say_searching ends" + ENDC)
#         return False
    
    rospy.loginfo('----> FALSE_TERM_CB')
    
    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['wave_recognition'] == 'succeeded':
        rospy.logwarn('Out_CB = Wave_Recognition succeeded') 
        return 'face_detected'
    
    elif outcome_map['wave_recognition'] == 'aborted':
        rospy.logwarn('Out_CB = Wave_Recognition succeeded') 
        return 'aborted'
    
    elif outcome_map['move_head'] == 'succeeded':
        rospy.logwarn('Out_CB = Find Faces Succeeded')
        return 'aborted'    
    
    elif outcome_map['Say_Searching'] == 'succeeded':
        rospy.logwarn('Out_CB = TimeOut finished succeeded')
        return 'aborted'   
    
    else:
        rospy.logwarn('Out_CB = Else!')
        return 'aborted'

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
                                   transitions={'succeeded': 'Concurrence', 'aborted': 'aborted', 
                                                'preempted': 'preempted', 'end_searching':'end_searching'})
            
            sm_conc = smach.Concurrence(outcomes = ['succeeded', 'aborted', 'preempted', 'face_detected'], 
                                        default_outcome = 'succeeded', 
                                        input_keys = ['head_left_right', 
                                                      'head_up_down'], 
                                        output_keys = ['wave_position', 'wave_yaw_degree'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            with sm_conc:
#                 sm_conc.StateMachine.add(
#                                        'move_head',
#                                        move_head_form(head_up_down=head_position),
#                                        transitions={'succeeded': 'Say_Searching',
#                                                     'preempted':'Say_Searching',
#                                                     'aborted':'aborted'})
#                 sm_conc.StateMachine.add(
#                     'Say_Searching',
#                     text_to_say(text_for_wave_searching, wait = False),
#                     transitions={'succeeded':'wave_recognition', 'aborted':'wave_recognition', 'preempted':'wave_recognition'})
                sm_conc.add(
                            'move_head',
                            move_head_form(head_up_down=head_position))
                sm_conc.add(
                            'Say_Searching',
                            text_to_say(text_for_wave_searching, wait = False))
                sm_conc.add(
                            'wave_recognition',
                            WaveDetection(8.0)) 
            
            smach.StateMachine.add('Concurrence',
                                   sm_conc,
                                   transitions={'succeeded':'Say_Found', 
                                                'aborted':'Move_head_prepare',
                                                'preempted':'preempted',
                                                'face_detected':'Say_Found'})
               
            #Hard-coded maximum time in order to detect wave 
#             smach.StateMachine.add(
#                 'wave_recognition',
#                 WaveDetection(4.0),
#                 transitions={'succeeded': 'Say_Found', 'aborted': 'Move_head_prepare', 
#                 'preempted': 'preempted'}) 
            
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








