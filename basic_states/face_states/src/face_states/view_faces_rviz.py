#!/usr/bin/env python
"""
Created on 6/06/14

@author: Sam Pfeiffer
"""

# system stuff
import sys
import copy

# ROS stuff
import rospy
from pal_detection_msgs.msg import FaceDetections, FaceDetection
from geometry_msgs.msg import Pose, PoseArray

global face_pub


def facecb(data):
    if len(data.faces) > 0:
        pa = PoseArray()
        pa.header = data.header
        for face in data.faces:
            print ".",
            face_pose = Pose()
            face_pose.position = face.position
            face_pose.orientation.w = 1.0
            pa.poses.append(face_pose)
        face_pub.publish(pa)


if __name__ == '__main__':
    rospy.init_node("view_faces_rviz")
    rospy.sleep(0.3)

    rospy.Subscriber('/pal_face/faces', FaceDetections, facecb)
    global face_pub
    face_pub = rospy.Publisher('/face_poses', PoseArray, latch=True)
    
    rospy.loginfo("Painting PoseArray of faces at /face_poses")
    rospy.spin()