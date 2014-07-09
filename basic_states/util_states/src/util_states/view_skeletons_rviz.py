#!/usr/bin/env python
"""
Created on 30/06/14

@author: Sam Pfeiffer
"""

# system stuff
import sys
import copy

# ROS stuff
import rospy
from pal_detection_msgs.msg import PersonDetections, PersonDetection
from geometry_msgs.msg import Pose, PoseArray

global skeleton_pub

def skeletoncb(data):
    if len(data.persons) > 0:
        pa = PoseArray()
        pa.header.frame_id = 'head_mount_xtion_depth_optical_frame' # by default is this one
        for person in data.persons:
            print "Skeleton at: " + str(person.position3D.point)
            skeleton_pose = Pose()
            skeleton_pose.position = person.position3D.point
            skeleton_pose.orientation.w = 1.0
            pa.poses.append(skeleton_pose)
        skeleton_pub.publish(pa)


if __name__ == '__main__':
    rospy.init_node("view_skeletons_rviz")
    rospy.sleep(0.3)

    rospy.Subscriber('/head_mount_xtion/users', PersonDetections, skeletoncb)
    global skeleton_pub
    skeleton_pub = rospy.Publisher('/skeleton_poses', PoseArray, latch=True)
    
    rospy.loginfo("Painting PoseArray of skeletons at /skeleton_poses")
    rospy.spin()