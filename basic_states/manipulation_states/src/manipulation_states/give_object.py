#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat April 5 11:30:00 2014

@author: Chang long Zhu
@email: changlongzj@gmail.com

"""


import rospy
import actionlib
import smach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from smach_ros.simple_action_state import SimpleActionState
from manipulation_states.play_motion_sm import play_motion_sm
from speech_states.say import text_to_say
from manipulation_states.move_hands_form import move_hands_form
     
class give_object(smach.StateMachine):
    """
    Executes a SM that: 
        Gives the object to the person.

    It should be called only when the --Grasp-- of the object has failed,
    which included the object_detection and the grasping per sé.
        
    Required parameters: None
    
    Optional parameters: None
    
    Input keys:
        None            
         
    Output keys:
        @key standard_error: Error
    """
        
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the Ask Process
            # Input Data: 'object_to_grasp'
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_lang = 'en_US'
            smach.StateMachine.add('Ask_Person_Object',
                                    text_to_say('I am giving you the object you asked'), 
                                    transitions={'succeeded':'Reach_Arm', 'preempted':'Reach_Arm', 'aborted':'Reach_Arm'})
            
            # Reach the arm
            self.userdata.manip_motion_to_play = 'give_object_right'
            self.userdata.manip_time_to_play = 2.0
            smach.StateMachine.add('Reach_Arm',
                                    play_motion_sm(),
                                    transitions={'succeeded':'Pre_Grasp', 'preempted':'Reach_Arm', 'aborted':'Reach_Arm'})

            smach.StateMachine.add('Pre_Grasp',
                                    move_hands_form(hand_pose_name='pre_grasp', hand_side='right'),
                                    transitions={'succeeded':'Open_Hand', 'preempted':'Open_Hand', 'aborted':'Open_Hand'})
            smach.StateMachine.add('Open_Hand',
                                    move_hands_form(hand_pose_name='full_open', hand_side='right'),
                                    transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
                            

