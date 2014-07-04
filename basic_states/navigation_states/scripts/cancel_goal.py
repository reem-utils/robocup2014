#! /usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalID
rospy.init_node("cancel_goal")
rospy.sleep(0.2)

poi_pub= rospy.Publisher('/move_base/cancel', GoalID,latch=True)

msg = GoalID()
poi_pub.publish(msg)
rospy.sleep(0.2)
poi_pub.unregister()
