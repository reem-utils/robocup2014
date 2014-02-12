#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: sampfeiffer
"""
import roslib; roslib.load_manifest('example_sm_pkg')
import rospy
import sys

from std_msgs.msg import String

class myNode():

    def __init__(self, argument_one):
        # my class variables
        self.my_variable_string = "I'm a string yo!"
        self.subs = rospy.Subscriber('my_topic_to_subscribe', String, self.myCallback)
        self.pub = rospy.Publisher('my_topic_to_publish', String, latch=True)
        self.myMethod()


    def myCallback(self, data):
        rospy.loginfo("Received from topic data!")
        self.myMethod()

    def myMethod(self):
        rospy.loginfo("Using the method!")
        publish_this_thing = String("I'm the content of a string!")
        self.pub.publish(publish_this_thing)


if __name__ == '__main__':
    rospy.init_node('node_name')
    if len(sys.argv) < 2:
        print "Error, we need an arg!"
        rospy.loginfo("No args given, closing...")
        exit()

    node = myNode("this is an argument")

    rospy.spin()