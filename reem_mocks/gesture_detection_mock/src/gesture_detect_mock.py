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


fakepositionx=0.5
fakepositiony=0.5
fakeorientation=0.5
information = True

class Gesture_Service():
	
	"""
		Gesture Detection Mock  

		Run the topic of face recognition:
			/gesture_detection/gesture

		Always recognize the wave gesture
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
		        mGesture.Gesture_name.data = 'wave'
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
