
#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Roger Boldu
@email: roger.boldu@gmail.com

"""

import rospy
import smach
from actionlib_msgs.msg import GoalID

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class cancel_nav(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             output_keys=[])
        #print "Setting publisher"
        self.poi_pub= rospy.Publisher('/move_base/cancel', GoalID)
        rospy.sleep(0.1)
        #print "Publisher set"
        
    def execute(self, userdata):
        msg = GoalID()
        self.poi_pub.publish(msg)
        return 'succeeded'
        
        

    
  


