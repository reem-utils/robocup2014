#! /usr/bin/env python

import rospy
import actionlib

from object_mock.msg import RecognizeAction, RecognizeResult, RecognizeGoal
from object_recognition_msgs.msg import ObjectType, RecognizedObject, RecognizedObjectArray

class object_detect_server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/blort_tracker/recognize_object', RecognizeAction, self.execute, False)
        self.server.start()
        
    def execute(self, goal):
        rospy.loginfo("Executing the server -- Goal Detected")
        result = RecognizeResult()
        possibleObjects = RecognizedObject()
        object_goal = goal.objects.pop()
        rospy.loginfo("Goal Received:: " + str(goal))
        rospy.loginfo("One of the Goals are:: " + str(object_goal))
        possibleObjects.type = object_goal
        
        rospy.loginfo("PossibleObject:: " + str(possibleObjects))
        
        objectArray = RecognizedObjectArray()
        objectArray.objects.append(possibleObjects)
        
        result.recognized_objects = objectArray 
        
        rospy.loginfo("Results that are sending to Object Detection SM :: " + str(result))
        rospy.sleep(goal.refine_pose_time)
        
            
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('object_detect_mock')
    server = object_detect_server()
    rospy.spin()