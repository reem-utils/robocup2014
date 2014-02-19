#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import play_motion

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from play_motion.msg import PlayMotionGoal, PlayMotionAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
 
def create_nav_goal(x, y, yaw):
    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
    Returns a MoveBaseGoal"""
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = '/map' # Note: the frame_id must be map
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
    
    # Orientation of the robot is expressed in the yaw value of euler angles
    angle = radians(yaw) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return mb_goal

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','exit'])
    
    #Init variables

    # Open the container
    with sm:        
        # Add states to the container
        nav_goal = create_nav_goal(-4.0, 4.0, 0.0)

        smach.StateMachine.add('WALK', SimpleActionState('/move_base', MoveBaseAction, nav_goal), 
                              transitions={'succeeded':'SALUTE'})
                              
                               
        #Salute state       
        goalPlay = PlayMotionGoal()
        
        goalPlay.motion_name = 'arms_t'
        goalPlay.reach_time.secs = 15
        smach.StateMachine.add('SALUTE', SimpleActionState('/play_motion', PlayMotionAction, goalPlay), 
                              transitions={'succeeded':'SALUTE2'})
        
#goalSound = SoundGoal()
 #       goalSound.string.data = sm.userdata.toSpeak_out
  #      smach.StateMachine.add('SPEAK', SimpleActionState('sound', SoundAction, goalSound),
   #                            transitions={'succeeded':'exit'})

        goalPlay = PlayMotionGoal()
        
        goalPlay.motion_name = 'home'
        goalPlay.reach_time.secs = 15
        smach.StateMachine.add('SALUTE2', SimpleActionState('/play_motion', PlayMotionAction, goalPlay), 
                              transitions={'succeeded':'exit'})
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
