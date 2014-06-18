#! /usr/bin/env python
'''
@author: Roger Boldu
@author: Sammy Pfeiffer

This class enables the robot to move straight
(forward or backwards) using odometry.
'''

import rospy

from geometry_msgs.msg import Twist, Pose
from odom_moving.srv import Straight, StraightResponse
from nav_msgs.msg import Odometry
from actionlib.simple_action_client import SimpleActionClient
import math
import copy


# We have safety in this node as we are using (implicitly) speed_limit to limit how fast
# we can move near any obstacle perceived by the sensors
CMD_VEL_TOPIC = '/key_vel'  # '/mobile_base_controller/cmd_vel'
ODOM_TOPIC = '/mobile_base_controller/odom'
SPEED_LIMIT_AS = '/speed_limit/disable'
STRAIGHT_SRV = '/straight_nav'

# This node parameters
MAXMETERS = 2.0
MAXTIME = 15.0
MINDIST = 0.15
SPEED_X = 0.1
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


class navigation_straight():
    """Class to provide moving striaght by odometry"""
    
    def __init__(self):
        rospy.loginfo("Initializing publisher to: '" + CMD_VEL_TOPIC + "'")
        self.nav_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist)
        
        rospy.loginfo("Subscribing to: '" + ODOM_TOPIC + "'")
        self.odom_subs = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_cb)
            
        rospy.loginfo("Setting forward service: '" + STRAIGHT_SRV + "'")
        self.nav_srv = rospy.Service(STRAIGHT_SRV, Straight, self.nav_straight_srv)
        self.init_vars()
        rospy.loginfo("Forward service up and running!")
        
    def init_vars(self):
        """Initialize class variables"""
        self.curr_odom = Odometry()
        self.init_odom = None
        self.enable = False        
        self.goal_achieved = False
        self.time_init = rospy.get_rostime()
        self.time_out = False
        
    def nav_straight_srv(self, req):
        """Callback for service requests"""
        rospy.loginfo("Got a request: " + str(req))
        if abs(req.meters) <= MAXMETERS:
            if req.enable:
                self.time_out = False
                self.time_init = rospy.get_rostime()
                self.enable = True
                self.meters = req.meters  # Number of meters that we have to move
                self.init_odom = self.curr_odom
                self.run()
                rospy.loginfo("Finished moving!")
                return StraightResponse()
            else:
                self.enable=False
                self.pub_stop()
                return StraightResponse()

        else:
            self.pub_stop()
            rospy.logwarn("Given distance is too long! Max distance is: " + str(MAXMETERS))
            return "Too much distance"  # this will fire up an error in the service call
        
        
    def odom_cb(self,data):
        """Odometry callback"""
        self.curr_odom = data
        
    def process_odometry(self):
        """Check how much we moved"""
        position_navigate=Pose()
        position_navigate.position.x = self.curr_odom.pose.pose.position.x - self.init_odom.pose.pose.position.x
        position_navigate.position.y = self.curr_odom.pose.pose.position.y - self.init_odom.pose.pose.position.y
        
        #unit_vector = normalize_vector(position_navigate.position)
        #print "unit vector: " + str(unit_vector)
        position_distance = vector_magnitude(position_navigate.position)
        #print "position_distance: " + str(position_distance)
        
        if position_distance < abs(self.meters):
            self.movment = False
        else:
            rospy.loginfo("We moved: " + str(position_distance) + "m of the desired " + str(self.meters) )
            self.movment = True
            
    def pub_move(self, direction):
        """Publish SPEED_X in a twist message"""
        msg = Twist()
        if direction == "forward":
            msg.linear.x = SPEED_X
        elif direction == "backward":
            msg.linear.x = - SPEED_X
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.nav_pub.publish(msg)

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

    def process_time(self):
        """Check if we are taking too long"""
        time_actual = rospy.get_rostime()
        time = time_actual.secs - self.time_init.secs
        
        if time < MAXTIME:
            self.time_out = False
        else:
            rospy.logwarn("Timeout!")
            self.time_out = True

    def run(self):
        """Run the movement"""
        rospy.loginfo("Moving!")
        rate = rospy.Rate(PUBLICATION_CMD_VEL_HZ)
        while self.enable: # While no one stops the movement
            self.process_odometry()
            self.process_time()
            if not self.movment and not self.time_out :
                if self.meters > 0.0:
                    self.pub_move('forward')
                else:
                    self.pub_move('backward')
            else:
                self.pub_stop()
                self.enable=False
            rate.sleep()
        
        
if __name__ == '__main__':
    rospy.init_node('navigation_straight_service')
    rospy.sleep(1)
    navigation = navigation_straight()
    rospy.spin()
