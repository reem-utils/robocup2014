#!/usr/bin/env python
"""
Created on 6/06/14

@author: Sam Pfeiffer
"""

# system stuff
import sys
import copy
from math import radians

# ROS stuff
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler


def usage():
    print "Usage:"
    print sys.argv[0] + " frame_id roll pitch yaw\n"
    print "Example: " + sys.argv[0] + " base_link 0.0 0.0 0.0"
    print "Use -d for using degrees"
    print "Example: " + sys.argv[0] + " base_link 0.0 0.0 0.0 -d"
    exit(0)

if __name__ == '__main__':
    if len(sys.argv) <= 4:
        usage()
    rospy.init_node("imidiot_i_need_to_see_rotations")
    rospy.sleep(0.3)
    
    if len(sys.argv) == 6:
        if sys.argv[5] == '-d':
            frame_id = sys.argv[1]
            roll = radians(float(sys.argv[2]))
            pitch = radians(float(sys.argv[3]))
            yaw = radians(float(sys.argv[4]))
    else:
        frame_id = sys.argv[1]
        roll = sys.argv[2]
        pitch = sys.argv[3]
        yaw = sys.argv[4]
    
    quat = quaternion_from_euler(float(roll), float(pitch), float(yaw))
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position = Point(2.0, 2.0, 2.0)
    p.pose.orientation = Quaternion(*quat)
    
    
    ori_pub = rospy.Publisher('/arrow_for_rotation', PoseStamped, latch=True)

    
    rospy.loginfo("Painting an arrow in arrow_for_rotation ")
    rospy.loginfo("Pose looks like: " + str(p))
    while not rospy.is_shutdown():
        ori_pub.publish(p)
        rospy.sleep(0.5)