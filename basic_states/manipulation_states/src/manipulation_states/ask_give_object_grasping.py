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

class prepare_ask_person_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            input_keys=['object_to_grasp'],
                            output_keys=['tts_text'],
                            outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        userdata.tts_text = 'Could you please give me the ' + userdata.object_to_grasp + '?'
        return 'succeeded'
        
class create_move_group_joints_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['manip_joint_names','manip_joint_group', 'manip_goal_joint_pose', 'move_it_joint_goal'],
                                    output_keys=['move_it_joint_goal','standard_error'])
    def execute(self, userdata):
       
        return 'succeeded'

class wait_state(smach.State):
    def __init__(self, time=3):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.wait_time = time
    def execute(self, userdata):
        rospy.sleep(self.wait_time)
        return 'succeeded'
        
class ask_give_object_grasping(smach.StateMachine):
    """
    Executes a SM that: 
        Asks the person to give a object ['object_to_grasp']. 
        The robot will extend its arm and open its hand.

    It should be called only when the --Grasp-- of the object has failed,
    which includesd the object_detection and the grasping per sé.
        
    Required parameters: None
    
    Optional parameters: None
    
    Input keys:
        @key object_to_grasp: indicates the object's name we want to grasp.            
         
    Output keys:
        @key standard_error: Error
    """
        
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['object_to_grasp'],
                                    output_keys=['standard_error'])
        with self:
            # Preparation of the Ask Process
            # Input Data: 'object_to_grasp'
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_lang = 'en_us'
            smach.StateMachine.add('Prepare_Ask_Person_Object',
                                    prepare_ask_person_object(),
                                    transitions={'succeeded':'Ask_Person_Object', 'preempted':'Ask_Person_Object', 'aborted':'Ask_Person_Object'})

            smach.StateMachine.add('Ask_Person_Object',
                                    text_to_say(), 
                                    transitions={'succeeded':'Reach_Arm', 'preempted':'Reach_Arm', 'aborted':'Reach_Arm'})
            
            # Reach the arm
            self.userdata.manip_motion_to_play = 'give_object_right'
            self.userdata.manip_time_to_play = 8.0
            smach.StateMachine.add('Reach_Arm',
                                    play_motion_sm(),
                                    transitions={'succeeded':'Open_hand', 'preempted':'Open_hand', 'aborted':'Open_hand'})
            
            # Open the hand
            smach.StateMachine.add('Open_hand', 
                                   move_hands_form(hand_pose_name='full_open', hand_side='right'),
                                   transitions={'succeeded':'Wait_for_object', 'preempted':'Wait_for_object', 'aborted':'Wait_for_object'})

            # Wait 5 seconds
            smach.StateMachine.add('Wait_for_object',
                                    wait_state(4),
                                    transitions={'succeeded':'Pre_Grasp', 'preempted':'Pre_Grasp', 'aborted':'Pre_Grasp'})

            smach.StateMachine.add('Pre_Grasp',
                                    move_hands_form(hand_pose_name='pre_grasp', hand_side='right'),
                                    transitions={'succeeded':'Full_Grasp', 'preempted':'Full_Grasp', 'aborted':'Full_Grasp'})
            smach.StateMachine.add('Full_Grasp',
                                    move_hands_form(hand_pose_name='grasp', hand_side='right'),
                                    transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})
                            

