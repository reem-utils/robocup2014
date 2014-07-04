#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun June 29 01:30:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import sys
import actionlib
import rospy
import smach

from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadResult, PointHeadActionGoal, PointHeadGoal
from move_joints_group import move_joints_group
from smach_ros.simple_action_state import SimpleActionState


POINTING_FRAME = 'head_mount_xtion_rgb_optical_frame'
#POINTING_FRAME = 'stereo_link'
TOPIC_ACTION = '/head_controller/point_head_action'

class prepare_move_head(smach.State):
    def __init__(self, point_to_look, frame_id):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['standard_error', 'target_frame_id', 'target_point'],
                             output_keys=['standard_error', 'point_goal'])
        self.s_point_to_look = point_to_look #[x, y, z]
        self.s_frame_id = frame_id #base_link...
    def execute(self, userdata):
        rospy.loginfo('Executing Move head')
        
        frame_id = self.s_frame_id if self.s_frame_id else userdata.target_frame_id
        point_to_look = self.s_point_to_look if self.s_point_to_look else userdata.target_point 
        
        look_goal = PointHeadGoal()
        look_goal.target.header.frame_id = frame_id
        look_goal.target.point.x = point_to_look[0]
        look_goal.target.point.y = point_to_look[1]
        look_goal.target.point.z = point_to_look[2]
        
        look_goal.pointing_frame = POINTING_FRAME 
        if POINTING_FRAME == 'head_mount_xtion_rgb_optical_frame':
            look_goal.pointing_axis.x = 0.0
            look_goal.pointing_axis.y = 0.0
            look_goal.pointing_axis.z = 1.0
        elif POINTING_FRAME == 'stereo_link':
            look_goal.pointing_axis.x = 1.0
            look_goal.pointing_axis.y = 0.0
            look_goal.pointing_axis.z = 0.0
        
        look_goal.min_duration.secs = 1.0
        
        rospy.loginfo('Sending this goal: ' + str(look_goal))
        
        userdata.point_goal = look_goal
               
        return 'succeeded'

        
class move_head_to_point(smach.StateMachine):
    
    """
    This SM moves the head in order to look at a specified point
    
    Required parameters: None
    
    Optional parameters: None
    
    Output keys: 
        @key standard_error: Error 
    
    Input keys:
        @key target_frame_id: the frame_id in which the Point Specified is based. Example: 'base_link', 'stereo_link'.
        @key target_point: the Point [x, y, z] for the robot to look at.
       
    """
    def __init__(self, point_to_look=None, frame_id=None):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['standard_error', 'target_frame_id', 'target_point'],
                                    output_keys=['standard_error', 'point_goal'])
        with self:
            # Preparation of the SM
            smach.StateMachine.add('prepare_move_head',
                                    prepare_move_head(point_to_look, frame_id), 
                                    transitions={'succeeded':'send_look_to_point', 'preempted':'preempted', 'aborted':'aborted'})

            def point_head_result_cb(self, error, point_info):
                print('Point Head Result: ' + str(point_info))
                print('ERROR: ' + str(error))
                
            smach.StateMachine.add('send_look_to_point', 
                                   SimpleActionState(TOPIC_ACTION, 
                                                     PointHeadAction, 
                                                     goal_key='point_goal',
                                                     exec_timeout=rospy.Duration(15.0), 
                                                     result_cb=point_head_result_cb, 
                                                     input_keys=['standard_error'], 
                                                     output_keys=['standard_error']),
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})

def main(args):
    rospy.init_node('look_to_point')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'end_searching'])
    with sm:
        smach.StateMachine.add(
            'gesture_state',
            move_head_to_point(point_to_look=[1, 1, 0], frame_id='stereo_link'),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()
    
if __name__ == '__main__':
    main(sys.argv)
    

