#! /usr/bin/env python
'''
@author: Roger Boldu
@author: Sammy Pfeiffer

This class enables the robot to move straight
(forward or backwards) using odometry.
'''

import rospy

from geometry_msgs.msg import Twist, Pose
from odom_moving.srv import Turn, TurnResponse
from nav_msgs.msg import Odometry
from actionlib.simple_action_client import SimpleActionClient
from tf.transformations import euler_from_quaternion
from tf import TransformListener
import math
import copy
import numpy as np

# We have safety in this node as we are using (implicitly) speed_limit to limit how fast
# we can move near any obstacle perceived by the sensors
CMD_VEL_TOPIC = '/key_vel'  # '/mobile_base_controller/cmd_vel'
TURN_SRV = '/turn_nav'

# This node parameters
MAXDEGREES = 180.0
MAXTIME = 15.0
SPEED_Z_ROT = 0.15
PUBLICATION_CMD_VEL_HZ = 20

# Functions borrowed from math_utils.py from Siegfried Gevatter
# Copied to keep this as simple as possible
def vector_magnitude(vec):
    """Returns the magnitude of the given vector."""
    return math.sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)

def normalize_vector(vec):
    """Returns a normalized (unitary) copy of *vec*."""
    magnitude = vector_magnitude(vec)
    unit = copy.deepcopy(vec)
    if magnitude > 0.0:
        unit.x /= magnitude
        unit.y /= magnitude
        unit.z /= magnitude
    return unit


class navigation_turn():
    """Class to provide moving turning by odometry"""
    
    def __init__(self):
        rospy.loginfo("Initializing publisher to: '" + CMD_VEL_TOPIC + "'")
        self.nav_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist)
        
        rospy.loginfo("Getting a TransformListener")   
        self.listener_ = TransformListener()
                
        rospy.loginfo("Setting turn service: '" + TURN_SRV + "'")
        self.nav_srv = rospy.Service(TURN_SRV, Turn, self.nav_turn_srv)
        self.init_vars()
        rospy.loginfo("Turn service up and running!")
        
        
        
    def init_vars(self):
        """Initialize class variables"""
        self.curr_odom = Odometry()
        self.init_odom = None
        self.enable = False        
        self.goal_achieved = False
        self.time_init = rospy.get_rostime()
        self.time_out = False
        
    def nav_turn_srv(self, req):
        """Callback for service requests"""
        rospy.loginfo("Got a request: " + str(req))
        if abs(req.degrees) <= MAXDEGREES:
            if req.enable:
                self.time_out = False
                self.time_init = rospy.get_rostime()
                self.enable = True
                self.degrees = req.degrees  # Number of degrees to rotate
                if self.turnOdom(math.radians(req.degrees)):
                    rospy.loginfo("Finished moving!")
                    return TurnResponse()
                else:
                    return "Timeout"
            else:
                self.enable=False
                self.pub_stop()
                return TurnResponse()

        else:
            self.pub_stop()
            rospy.logwarn("Given turn is too big! Max distance is: " + str(MAXDEGREES))
            return "Too much turn"  # this will fire up an error in the service call
        

    def pub_stop(self):
        """Publish 0.0's in a twist message"""
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.nav_pub.publish(msg)

    def is_timeout(self):
        """Check if we are taking too long"""
        time_actual = rospy.get_rostime()
        time = time_actual.secs - self.time_init.secs
        if time < MAXTIME:
            return False
        else:
            rospy.logwarn("Timeout!")
            return True


        
    def turnOdom(self, radians):
        """Turn radians.
        Based on http://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20robot%20base%20controllers%20to%20drive%20the%20robot"""
        if radians > 0.0:
            clockwise = True
        else:
            clockwise = False
        
        #wait for the listener to get the first message
        self.listener_.waitForTransform("base_footprint", "odom", rospy.Time(0), rospy.Duration(3.0))
        
        #record the starting transform from the odometry to the base frame
        (trans, start_rot) = self.listener_.lookupTransform("base_footprint", "odom", rospy.Time(0))
        roll, pitch, start_yaw = euler_from_quaternion(start_rot)
         
        # Command to send to turn
        base_cmd = Twist()
        base_cmd.linear.x = base_cmd.linear.y = 0.0
        base_cmd.angular.z = SPEED_Z_ROT
        if clockwise:
            base_cmd.angular.z = -base_cmd.angular.z
        
        rate = rospy.Rate(20.0)
        done = False
        last_yaw = None
        while not done and not rospy.is_shutdown():
            # Send the drive command
            self.nav_pub.publish(base_cmd)
            if self.is_timeout():
                rospy.logwarn("Timeout rotating")
                return False
            rate.sleep()
            # Get the current transform
            try:
                #current_transform = listener_.lookupTransform("base_footprint", "odom", rospy.Time(0))
                (curr_trans, curr_rot) = self.listener_.lookupTransform("base_footprint", "odom", rospy.Time(0))
            except:
                rospy.logerr("Transform exception")
        
            #print "start_yaw : " + str(start_yaw)    
            roll, pitch, curr_yaw = euler_from_quaternion(curr_rot)
            #print "curr_yaw : " + str(curr_yaw)
            
            if last_yaw == None:
                last_yaw = curr_yaw
            else:
                if clockwise:
                    if (cmp(last_yaw, 0.0) == 1) and (cmp(curr_yaw, 0.0) == -1):
                        # if the sign of last_yaw and curr_yaw is not the same means
                        # that we overflowed over pi so must fix that
                        # This means that rotating clockwise we go
                        # -pi ... 0.0 ... pi
                        #       ------>
                        # So a jump from pi to -pi will happen
                        print "Overflowed!"
                        curr_yaw = 2 * math.pi + curr_yaw
                    else:
                        last_yaw = curr_yaw
                else:
                    if (cmp(last_yaw, 0.0) == -1) and (cmp(curr_yaw, 0.0) == 1):
                        # same comparisson than up but for the other way around
                        # This means that rotating counter clockwise we go
                        # -pi ... 0.0 ... pi
                        #       <------
                        # So a jump from -pi to pi will happen
                        print "Underflowed!"
                        curr_yaw = - 2 * math.pi + curr_yaw
                    else:
                        last_yaw = curr_yaw
            
            relative_yaw = start_yaw - curr_yaw
#             print "relative yaw: " + str(relative_yaw)
#             print "target_yaw: " + str(radians)
            if abs(relative_yaw) > abs(radians):
                done = True
                rospy.loginfo("We turned: " + str(math.degrees(relative_yaw)) + " from the " + str(math.degrees(radians * -1)) + " requested")
            if done:
                return True
        return False


        
if __name__ == '__main__':
    rospy.init_node('navigation_turn_service')
    rospy.sleep(1)
    navigation = navigation_turn()
    #navigation.turnOdom(-1.57 / 2.0)
    rospy.spin()
