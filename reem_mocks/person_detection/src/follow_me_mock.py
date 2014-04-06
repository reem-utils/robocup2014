#! /usr/bin/env python
'''
@author: Roger Boldu
'''


import rospy
import actionlib
import sys
import select

from pal_detection_msgs.srv import Recognizer
from pal_detection_msgs.srv import RecognizerRequest, RecognizerResponse
from pal_detection_msgs.msg import FaceDetection, FaceDetections
from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest, StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse
from pal_detection_msgs.srv import SetDatabase, SetDatabaseRequest, SetDatabaseResponse
from follow_me.msg import tracker_people

fakepositionx=0
fakepositiony=2
fakeorientation=0
information=True
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class Follow_me_Service():
	"""
	Follow me Mock 

	Run the topic of face recognition:
		/people_tracker/person

	"""
	def __init__(self):
		rospy.loginfo("Initializing tracker_people_service")
		self.person_pub=rospy.Publisher('/people_tracker/person',tracker_people)
	def isData(self):
	  return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])  
	 
	def run(self):
		"""Publishing usersaid when face recognitionL is enabled """
		# TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
		while not rospy.is_shutdown(): 
			if self.isData():
				rospy.loginfo("i have new key")
				c = sys.stdin.read(1) #Clear buffer
				if c == 'w':
				    fakepositionx=1
				elif c == 'a':
					fakepositiony =1
				elif c == 'd':
					fakepositiony=1
				else:
					fakepositionx=0
					fakepositiony=0
					
			else:	
				rospy.loginfo("NOOOOOO key")
				fakepositionx=0
				fakepositiony=0
			if information:
				person=tracker_people()
				person.confidence=100
				person.pose.position.x=fakepositionx
				person.pose.position.y=fakepositiony
				person.pose.orientation.w=fakeorientation
			else :
				person=tracker_people()
			rospy.loginfo(OKGREEN+"im sending:     "+str(person)+ENDC)
			self.person_pub.publish(person)
			
			if 	person.pose.position.x==0 and person.pose.position.y==0:
				rospy.loginfo("NO CHANGES")
				rospy.sleep(0.5)
			else:
				rospy.sleep(3)


if __name__ == '__main__':
	rospy.init_node('follow_me_srv')
	rospy.loginfo("Initializing follow_me_srv")
	person=Follow_me_Service()
	
	person.run()
    
    
  
