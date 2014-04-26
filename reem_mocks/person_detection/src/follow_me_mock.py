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
from follow_me.msg import personArray, person

fakepositionx=0
fakepositiony=0
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
		self.person_pub=rospy.Publisher('/people_tracker_node/peopleSet',personArray)
	def isData(self):
	  return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])  
	 
	def run(self):
		"""Publishing usersaid when face recognitionL is enabled """
		# TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
		PersonArray=personArray()
		pers=person()
		pers.x=0
		pers.y=1
		pers.targetId=1
		pers.status=4
		PersonArray.peopleSet.append(pers)
		pers=person()
		pers.x=1
		pers.y=0
		pers.targetId=1
		pers.status=4
		PersonArray.peopleSet.append(pers)
		while not rospy.is_shutdown(): 
			self.person_pub.publish(PersonArray)
		rospy.sleep(0.1)# publish a 10 Hz



if __name__ == '__main__':
	rospy.init_node('follow_me_srv')
	rospy.loginfo("Initializing follow_me_srv")
	follow=Follow_me_Service()
	
	follow.run()
    
    
  
