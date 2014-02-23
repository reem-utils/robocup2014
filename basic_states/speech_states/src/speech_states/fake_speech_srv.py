#! /usr/bin/env python

import roslib;# roslib.load_manifest('exaction')
import rospy
import actionlib
#import sound_play.msg  ## revisar com funciona sound_play si vols que PC parly tmb

from text_to_speech.msg import SoundAction, SoundGoal, SoundResult #NO SE SI EXISTEIX 

class DoDishesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('/sound', SoundAction, self.execute, False)
    self.server.start()
    rospy.loginfo("Server running, waiting for goals")

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    rospy.loginfo(goal.text);
    result = SoundResult()
    result = 0
    
    #pub = rospy.Publisher('robotsound', sound_play.msg.SoundRequest)    
    #msound = sound_play.msg.SoundRequest(-3, 1, goal.text, '')    
    #pub.publish(msound)
    
    self.server.set_succeeded(result)


if __name__ == '__main__':
  rospy.init_node('dishes_srv')
  rospy.loginfo("Initializing dishes_srv")
  server = DoDishesServer()
  rospy.spin()
