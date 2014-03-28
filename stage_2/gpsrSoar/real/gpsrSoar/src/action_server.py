#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

from generate_goal import printNewGoal 

import actionlib

from gpsr.msg import order_list

class gpsrASAction(object):

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(name, orderList, execute_cb=self.execute_cb)
    self._as.start()
    self._hasgoal = False
    
  def execute_cb(self, goal):
    i=0
    for commands in goal:
      i += 1
      action = goal.action 
      loc = goal.location
      it = goal.item
      pers = goal.person

      printNewGoal(action, it, pers, loc, i)
    
    for i in xrange(1, goal.order):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
"""
      self._feedback.order_id = i
      self._as.publish_feedback(self._feedback)
"""   
      
    self.hasgoal = True
  
  def has_succeeded(self):
    self._result.order_id  = goal.order
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('gpsrAS')
  gpsrASAction(rospy.get_name())
  rospy.spin()
