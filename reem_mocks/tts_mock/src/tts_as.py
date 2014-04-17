#! /usr/bin/env python

import rospy
import actionlib

#from text_to_speech.msg import SoundAction, SoundGoal, SoundResult 
from pal_interaction_msgs.msg import SoundAction, SoundGoal, SoundResult 

class TtsActionServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('/sound', SoundAction, self.execute, False)
    self.server.start()
    rospy.loginfo("ActionServer running, waiting for goals")

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    rospy.sleep(goal.wait_before_speaking)
    rospy.loginfo(goal.lang_id)
    rospy.loginfo(goal.text)
    result = SoundResult()
    result.text = goal.text
        
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('tts_as')
  rospy.loginfo("Initializing tts_as")
  server = TtsActionServer()
  rospy.spin()
