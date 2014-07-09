#! /usr/bin/env python
'''
@author: Roger Boldu
@author: Sammy Pfeiffer

'''

import rospy
from actionlib import SimpleActionServer
from follow_me.msg import PipolFollowAction, PipolFollowFeedback, PipolFollowResult

PIPOL_FOLLOWER_AS = '/pipol_follower_as'

class PipolFollower():
    def __init__(self):
        
        rospy.loginfo("Creating Pipol follower AS: '" + PIPOL_FOLLOWER_AS + "'")
        self._as = SimpleActionServer(PIPOL_FOLLOWER_AS, PipolFollowAction, 
                                      execute_cb = self.execute_cb,
                                      preempt_callback = self.preempt_cb, 
                                      auto_start = False)
        rospy.loginfo("Starting " + PIPOL_FOLLOWER_AS)
        self._as.start()
        
    def execute_cb(self, goal):
        print "goal is: " + str(goal)
        # helper variables
        success = True
        
        # start executing the action
        for i in xrange(1, goal.order):
          # check that preempt has not been requested by the client
          if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            break
          self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
          # publish the feedback
          self._as.publish_feedback(self._feedback)
          # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
          r.sleep()
          
        if success:
          self._result.sequence = self._feedback.sequence
          rospy.loginfo('%s: Succeeded' % self._action_name)
          self._as.set_succeeded(self._result)  


if __name__ == '__main__':
    rospy.init_node('pipol_follower_node')
    rospy.sleep(1)