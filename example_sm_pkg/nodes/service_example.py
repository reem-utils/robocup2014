#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:12:48 2013

@author: sampfeiffer
"""

#Service example:
#http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
#Which needs that you have completed (did you?):
#http://wiki.ros.org/ROS/Tutorials/CreatingPackage and http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv


import roslib; roslib.load_manifest('beginner_tutorials')

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()