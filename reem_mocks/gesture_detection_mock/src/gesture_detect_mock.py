#! /usr/bin/env python
"""
@author Chang Long Zhu Jin
@email changlongzj@gmail.com
"""

import rospy
import actionlib
import sys
import select


from gesture_detection_mock.msg import Gesture
#from grasping_mock.msg import ObjectDetection


fakepositionx=1
fakepositiony=2
fakeorientation=1.6
information = True

class Gesture_Service():
    """
    Gesture Detection Mock
    It is a mock service, in which it sends a Gesture MSG (Pose) and the name of the detected geture.

    TODO: The position should be changing in a constant time.
    """

    def __init__(self):
        rospy.loginfo("Initializing gesture_detection")
        self.mGesture_pub=rospy.Publisher('/gesture_detection/gesture',Gesture)
         
    def run(self):

        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
               
            if information:
                #people
                mGesture=Gesture()
                mGesture.Gesture_name.data = 'Wave'
                mGesture.Gesture_id = 1
                mGesture.gesture_position.position.x=fakepositionx
                mGesture.gesture_position.position.y=fakepositiony
                mGesture.gesture_position.orientation.w=fakeorientation
            
            else :
                mGesture=Gesture()
                
            self.mGesture_pub.publish(mGesture)
                    
            rospy.sleep(5)
        
        
if __name__ == '__main__':
    rospy.init_node('gesture_srv')
    rospy.loginfo("Initializing gesture_srv")
    sGesture=Gesture_Service()
   
    sGesture.run()
