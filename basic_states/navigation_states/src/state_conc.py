#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import play_motion
import sys          #required for arguments

from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from play_motion.msg import PlayMotionGoal, PlayMotionAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
 
#define state Prepare        
class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endPrepare'], input_keys=['xWalk_in'], output_keys=['xWalk_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Prepare')
        userdata.xWalk_out = 1.0
        return 'endPrepare'

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
def main(argv):
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','exit'])
    
    #Init variables
    sm.userdata.xWalk = 0.0

    # Open the container
    with sm:        
        # Add states to the container

        # Data Passing 
        smach.StateMachine.add('Prepare', Prepare(), 
                                transitions={'endPrepare':'Concurrence'},
                                remapping={'xWalk_in':'xWalk', 'xWalk_out':'xWalk'})
        
        # Concurrence
        sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted', 'exit'],
                                    default_outcome='succeeded',
                                    outcome_map={'succeeded': {'WALK': 'succeeded', 'MotionArms': 'succeeded'}})

        with sm_conc:
            nav_goal = create_nav_goal(sm.userdata.xWalk, float(argv[1]), 0.0)
            
            smach.Concurrence.add('WALK', SimpleActionState('/move_base', MoveBaseAction, nav_goal))                  
      
            goalPlay = PlayMotionGoal()
        
            goalPlay.motion_name = 'arms_t'
            goalPlay.reach_time.secs = 15
            smach.Concurrence.add('MotionArms', SimpleActionState('/play_motion', PlayMotionAction, goalPlay))


        smach.StateMachine.add('Concurrence', sm_conc, 
                                transitions={'succeeded':'MotionHome'})

        goalPlay = PlayMotionGoal()
        
        goalPlay.motion_name = 'home'
        goalPlay.reach_time.secs = 10
        smach.StateMachine.add('MotionHome', SimpleActionState('/play_motion', PlayMotionAction, goalPlay), 
                              transitions={'succeeded':'exit'})
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print "Number of params incorrect"
    else:
        main(sys.argv)
    
