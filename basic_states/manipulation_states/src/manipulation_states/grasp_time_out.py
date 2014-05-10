#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat May 10 11:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com

"""


import rospy
import actionlib
import smach
import smach_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from smach_ros.simple_action_state import SimpleActionState
from manipulation_states.play_motion_sm import play_motion_sm
from speech_states.say import text_to_say
from manipulation_states.move_hands_form import move_hands_form

import time

time_First = True

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[]) 

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(10)
        return 'succeeded'
    
class Time_State(smach.State):
    def __init__(self, ttl=20.0):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'time_out', 'preempted'],
                             input_keys=['time_grasp', 'time_first', 'time_out_grasp'],
                             output_keys=['time_grasp', 'time_first', 'time_out_grasp'])
        
        self.time_to_live = ttl
    def execute(self, userdata):
#         if userdata.time_first == True :
#             userdata.time_first = False
        userdata.time_out_grasp = self.time_to_live if self.time_to_live else userdata.time_out_grasp
        userdata.time_grasp = rospy.get_rostime()
        
        while (rospy.get_rostime().secs - userdata.time_grasp.secs) < userdata.time_out_grasp :
            rospy.sleep(0.3)
            if self.preempt_requested():
                return 'preempted'
        
        return 'time_out'
        

def child_term_cb(outcome_map):
    if outcome_map['Time_State'] == 'time_out':
        return True
    
    if outcome_map['Find_and_grab_object'] == 'succeeded':
        return True
    
    return False
        
def out_cb(outcome_map):
    if outcome_map['Time_State'] == 'time_out':
        return 'time_out'
    if outcome_map['Find_and_grab_object'] == 'succeeded':
        return 'succeeded'
    return 'aborted'


class grasping_with_timeout(smach.StateMachine):
    """
    Executes a SM that: 
        Executes the Grasping SM (Object detection + Grasping)
        Calculates the time_out.
        
    Required parameters: None
    
    Optional parameters: None
    
    Input keys:
        @key object_to_grasp: indicates the object's name we want to grasp.            
        @key time_out_grasp: indicates the maximum grasping time
    Output keys:
        @key standard_error: Error
    """
        
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'time_out'], 
                                    input_keys=['object_to_grasp', 'time_out_grasp'],
                                    output_keys=['standard_error'])
        with self:

            self.userdata.time_grasp = 0.0
            self.userdata.time_first = True
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'time_out'],
                                        default_outcome='succeeded',
                                        input_keys=['object_to_grasp', 'time_grasp', 'time_first', 'time_out_grasp'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb = out_cb)

            with sm_conc:
                sm_conc.add(
                    'Find_and_grab_object',
                    #Find_and_grab_object(),
                    DummyStateMachine())
                
                sm_conc.add(
                            'Time_State',
                            Time_State(ttl=5.0))
                
            smach.StateMachine.add('GRASP_CONCURRENCE',
                                   sm_conc,
                                   transitions={'succeeded':'succeeded',
                                                'time_out':'time_out'})


if __name__=='__main__':
    rospy.init_node('grasp_time_out')
    sm = smach.StateMachine(outcomes=['succeeded', 'time_out'])
    with sm:
        sm.userdata.time_out_grasp = 5.0
        sm.userdata.object_to_grasp = 'coke'
        smach.StateMachine.add('Grasp_time', 
               grasping_with_timeout(), 
               transitions={'succeeded':'succeeded', 'time_out':'time_out'})
    sis = smach_ros.IntrospectionServer('cocktail_party',
                                        sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()