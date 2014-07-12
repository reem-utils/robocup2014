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
import tf


from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# Constants


class send_goal(smach.State):
    def __init__(self,coord_pub):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['navigation_goal'], output_keys=[])
        self.coord_pub=coord_pub
    def execute(self, userdata):
        
        self.coord_pub.publish(userdata.navigation_goal)
        
        return 'succeeded'
    
    
class createNavGoal(smach.State):
    def __init__(self, frame_id, tf_listener):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_coord_goal'], output_keys=['navigation_goal'])
        self.frame_id = frame_id
        self.tf_listener = tf_listener
        
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
        latest_common_tf_time = self.tf_listener.getLatestCommonTime("odom", "base_link")
        mb_goal.header.stamp = latest_common_tf_time
        
        mb_goal.header.frame_id = self.frame_id # Note: the frame_id must be map
        mb_goal.pose.position.x = x
        mb_goal.pose.position.y = y
        mb_goal.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
         
        # Orientation of the robot is expressed in the yaw value of euler angles
        quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw
        mb_goal.pose.orientation = Quaternion(*quat.tolist())
        
        transform_ok = False
        while not transform_ok: # this is ugly as is polling a lot to TF... but works
            try:
                transformed_map_pose = self.tf_listener.transformPose("odom", mb_goal)
                transform_ok = True
            except tf.ExtrapolationException:
                rospy.logwarn("Exception on transforming transformed_pose... trying again.")
                mb_goal.header.stamp = rospy.Time.now()
        mb_goal.header.stamp = rospy.Time.now() + rospy.Duration(5.0)
        return transformed_map_pose

class nav_to_coord_concurrent(smach.StateMachine):
    """
    Navigate to given map coords.

    This SM navigates to a given coordenates using the parameter
    This input data should be a (x,y,yaw) point
    It works like nav_to_coord, the difference is that you don need to wait the final
    
    @input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
    rotation in radians
    @output_keys: standard_error string representing the possible error
    @optional you can put a frame_id, if you don't put it will be the /map
   """

    
    def __init__(self,frame_id='/map'):
        """
        Constructor for nav_to_coord.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['nav_to_coord_goal'],
                                 output_keys=['standard_error'])
        self.coord_pub= rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True)
        rospy.loginfo("Getting a TransformListener...")
        self.tf_listener = tf.TransformListener()
        
        with self: 
            self.userdata.standard_error='OK'
            smach.StateMachine.add('CreateNavGoal',
                                   createNavGoal(frame_id, self.tf_listener),
                                   transitions={'succeeded':'SEND_GOAL', 'aborted':'aborted'})
            
            smach.StateMachine.add('SEND_GOAL',
                       send_goal(self.coord_pub),
                       transitions={'succeeded':'succeeded'})
            
        
            


