#! /usr/bin/env python

'''
@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
'''

# System stuff
import sys

# ROS stuff
import rospy

# SMACH stuff
import smach
import smach_ros
from smach_ros import SimpleActionState
# Msgs
from pipol_tracker_pkg.msg import person, personArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Int32
from control_msgs.msg import PointHeadActionGoal, PointHeadAction

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

POINT_HEAD_TOPIC = '/head_controller/point_head_action/goal'

TIME_BETWEEN_GOALS = 0.3

class prepare_data_look(smach.State):
    def __init__(self, point_to_look, frame_id):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                              input_keys=['point_to_look'],
                              output_keys=['point_head_goal'])
        
        self.point_to_look = point_to_look
        self.frame_id = frame_id
          
    def execute(self, userdata):
        
        if userdata.point_to_look == None and self.point_to_look == None:
            rospy.logerr("No point to look at! Error at SM Look_to_point")
            return 'aborted'
        
        self.point_to_look = self.point_to_look if self.point_to_look else userdata.point_to_look
        
        phg = PointHeadActionGoal()
        phg.goal.min_duration = rospy.Duration(0.6) # adapt for as far as the detection is??
        
        phg.goal.target.header.frame_id = self.frame_id if self.frame_id else self.point_to_look.header.frame_id
        
        phg.goal.target.header.stamp = rospy.Time.now()
        
        phg.goal.target.point.x = self.point_to_look.point.x
        phg.goal.target.point.y = self.point_to_look.point.y
        phg.goal.target.point.z = self.point_to_look.point.z
        
        phg.goal.pointing_axis.x = 1.0
        phg.goal.pointing_frame = 'head_mount_xtion_rgb_frame'
        
        userdata.point_head_goal = phg
        
        return 'succeeded'
    
class look_to_point(smach.StateMachine):
    """
    This SM makes the robot to look at a defined Point (in 3D space).
    It needs the Point (PointStamped) with X, Y, Z and Frame_ID; or another Frame_id if necesary.
    
    Optional Parameters:
        @param point_to_look: PointStamped
        @param frame_id: frame_id of the Point
    
    Input Keys:
        @key point_to_look: PointStamped
          
    """
    
    def __init__(self, point_to_look = None, frame_id = None):
        smach.StateMachine.__init__(self, 
                                    input_keys = ['point_to_look'],
                                    output_keys = ['standard_error'],
                                    outcomes=['succeeded', 'preempted', 'aborted'])

        with self:
            self.userdata.standard_error = " "
            
            smach.StateMachine.add(
                 'prepare_data',
                 prepare_data_look("I'm searching for people waving at me", wait=False),
                 transitions={'succeeded': 'wave_recognition', 'aborted': 'wave_recognition'}) 
            
            def look_to_point_cb(userdata, result_status, result):
                if result_status != 3: # 3 == SUCCEEDED
                    rospy.logwarn('Error in Look_to_Point: ' + str(result))
                    if result_status == 4: 
                        userdata.standard_error = "Aborted looking goal"
                        rospy.loginfo(userdata.standard_error)
                    elif result_status == 2:
                        return 'preempted'
                    return 'aborted'
                else:
                    userdata.standard_error = "OK"
                return 'succeeded'

            smach.StateMachine.add('MoveRobot', 
                                SimpleActionState(POINT_HEAD_TOPIC,
                                                   PointHeadAction,
                                                   goal_key='point_head_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=look_to_point_cb), 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'aborted', 
                                             'preempted':'preempted'})
            
            
            
            