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
from object_recognition_msgs.msg import TableArray, Table
from geometry_msgs.msg import Pose, PoseArray

global tables_pub

def tablecb(data):
    if len(data.tables) > 0:
        pa = PoseArray()
        pa.header.frame_id = 'head_mount_xtion_rgb_optical_frame' # by default is this one
        print "data is: " + str(data)
        print "\n\n-----"
        for table in data.tables:
            print "table at: " + str(table.pose)
            table_pose = Pose()
            table_pose = table.pose
            pa.poses.append(table_pose)
        tables_pub.publish(pa)


if __name__ == '__main__':
    rospy.init_node("view_tables_rviz")
    rospy.sleep(0.3)

    rospy.Subscriber('/table_array', TableArray, tablecb)
    global tables_pub
    tables_pub = rospy.Publisher('/table_poses', PoseArray, latch=True)
    
    rospy.loginfo("Painting PoseArray of tables at /table_poses")
    rospy.spin()