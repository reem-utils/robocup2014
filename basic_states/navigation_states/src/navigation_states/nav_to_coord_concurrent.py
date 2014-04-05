#!/usr/bin/env python

"""
@author:  Roger Boldu
@email: roger.boldu@gmail.com

"""


#import rospy
import rospy
import smach
from smach_ros import SimpleActionState
from geometry_msgs.msg import PoseStamped



from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# Constants
'''
It works like nav_to_coord, the difference is that you don need to wait the final
@ input : navigation_goal= x, y, yaw
@ No output
@ optional you can put a frame_id, if you don't put it will be the /map
'''


class send_goal(smach.State):
    def __init__(self,coord_pub):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['navigation_goal'], output_keys=[])
        self.coord_pub=coord_pub
    def execute(self, userdata):
        
        self.coord_pub.publish(userdata.navigation_goal)
        
        return 'succeeded'
    
    
class createNavGoal(smach.State):
    def __init__(self, frame_id):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_coord_goal'], output_keys=['navigation_goal'])
        self.frame_id = frame_id
        
    def execute(self, userdata):
        nav_goal = self.create_nav_goal(userdata.nav_to_coord_goal[0], 
                                    userdata.nav_to_coord_goal[1], 
                                    userdata.nav_to_coord_goal[2])

        userdata.navigation_goal = nav_goal
        return 'succeeded'
 
    def create_nav_goal(self, x, y, yaw):
        """Create a MoveBaseGoal with x, y position and yaw rotation (in radians).
        Returns a MoveBaseGoal"""
        mb_goal = PoseStamped()
        mb_goal.header.frame_id = self.frame_id # Note: the frame_id must be map
        mb_goal.pose.position.x = x
        mb_goal.pose.position.y = y
        mb_goal.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
         
        # Orientation of the robot is expressed in the yaw value of euler angles
        quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw
        mb_goal.pose.orientation = Quaternion(*quat.tolist())

        return mb_goal

class nav_to_coord_concurrent(smach.StateMachine):
    """
    Navigate to given map coords.

    This SM navigates to a given coordenates using the parameter
    This input data should be a (x,y,yaw) point
    
    @input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
    rotation in radians
    @output_keys: standard_error string representing the possible error
    """

    
    def __init__(self,frame_id='/map'):
        """
        Constructor for nav_to_coord.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['nav_to_coord_goal'],
                                 output_keys=['standard_error'])
        self.coord_pub= rospy.Publisher('/move_base_simple/goal', PoseStamped)
        with self: 

            smach.StateMachine.add('CreateNavGoal',
                                   createNavGoal(frame_id),
                                   transitions={'succeeded':'SEND_GOAL', 'aborted':'aborted'})
            
            smach.StateMachine.add('SEND_GOAL',
                       send_goal(self.coord_pub),
                       transitions={'succeeded':'succeeded'})
            
        
            


