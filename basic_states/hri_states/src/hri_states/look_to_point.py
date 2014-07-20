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
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Int32
from control_msgs.msg import PointHeadActionGoal, PointHeadAction, PointHeadGoal

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

POINT_HEAD_TOPIC = '/head_controller/point_head_action'

TIME_BETWEEN_GOALS = 0.3
UP=1.3

class prepare_data_look(smach.State):
    def __init__(self, point_to_look, frame_id, min_duration=0.9,direction=""):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                              input_keys=['point_to_look'],
                              output_keys=['point_head_goal'])
        
        self.point_to_look = point_to_look
        self.frame_id = frame_id
        self.min_duration = min_duration
        self.direction=direction
        
          
    def execute(self, userdata):
        
        if userdata.point_to_look == None and self.point_to_look == None:
            rospy.logerr("No point to look at! Error at SM Look_to_point")
            return 'aborted'
        print str(self.direction)
        
        self.point_to_look = self.point_to_look if self.point_to_look else userdata.point_to_look
        
        phg = PointHeadGoal()
        phg.min_duration = rospy.Duration(self.min_duration) # adapt for as far as the detection is??
        
        phg.target.header.frame_id = self.frame_id if self.frame_id else self.point_to_look.header.frame_id
        
        phg.target.header.stamp = rospy.Time.now()
        
        phg.target.point.x = self.point_to_look.point.x
        phg.target.point.y = self.point_to_look.point.y
        phg.target.point.z = self.point_to_look.point.z
        
        phg.pointing_axis.x = 1.0
        phg.pointing_frame = 'head_mount_xtion_rgb_frame'
        
        if self.direction!="" :
            print "I HAVE A NEW ORDER"
            phg.target.point.z=UP
            if self.direction =="left" :
                phg.target.point.y = 1
            elif self.direction=="right":
                phg.target.point.y = -1
            elif self.direction=="front":
                phg.target.point.y = 0
            elif self.direction=="up":
                phg.target.point.y = 0
                phg.target.point.z=1.6
        
        
        
        userdata.point_head_goal = phg
        print phg
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
    
    def __init__(self, point_to_look = None, frame_id = None, direction = "",min_duration=0.9):
        smach.StateMachine.__init__(self, 
                                    input_keys = ['point_to_look'],
                                    output_keys = ['standard_error'],
                                    outcomes=['succeeded', 'preempted', 'aborted'])
        
        with self:
            self.userdata.standard_error = " "
            self.userdata.point_to_look = PointStamped()
            self.userdata.point_to_look.header.frame_id = 'base_link'
            self.userdata.point_to_look.point.x = 1.0
            self.userdata.point_to_look.point.y = 0.0
            self.userdata.point_to_look.point.z = 1.0
            
            
#             smach.StateMachine.add('look_direction', 
#                     direction_calculate(direction,min_duration), 
#                     transitions={'succeeded':'prepare_data', 
#                                  'aborted':'prepare_data', 
#                                  'preempted':'preempted'})

            
            
            smach.StateMachine.add(
                 'prepare_data',
                 prepare_data_look(point_to_look, frame_id,min_duration=0.9,direction=direction),
                 transitions={'succeeded': 'look_to_point', 'aborted': 'aborted'}) 
            
            
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
            


            smach.StateMachine.add('look_to_point', 
                                SimpleActionState(POINT_HEAD_TOPIC,
                                                   PointHeadAction,
                                                   goal_key='point_head_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=look_to_point_cb), 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'aborted', 
                                             'preempted':'preempted'})
            
            
def main():
    rospy.loginfo('Look To Point')
    rospy.init_node('look_to_point_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                            input_keys=[])
    with sm:      
        sm.userdata.point_to_look = PointStamped()
        sm.userdata.point_to_look.header.frame_id = 'base_link'
        sm.userdata.point_to_look.point.x = 1.0
        sm.userdata.point_to_look.point.y = 1.0
        sm.userdata.point_to_look.point.z = 1.0
        
        smach.StateMachine.add(
            'look_to_point',
            look_to_point(direction="left",min_duration=2.0),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()        