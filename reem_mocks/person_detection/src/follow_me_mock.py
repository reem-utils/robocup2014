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
fakeorientation=1.6
information=True

class Follow_me_Service():
	"""
	Follow me Mock 

	Run the topic of face recognition:
		/people_tracker/person

	"""
    
    def __init__(self):
        rospy.loginfo("Initializing tracker_people_service")
        self.person_pub=rospy.Publisher('/people_tracker/person',tracker_people)
         
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
               
            if information:
                #people
                person=tracker_people()
                person.confidence=100
                person.pose.position.x=fakepositionx
                person.pose.position.y=fakepositiony
                person.pose.orientation.w=fakeorientation
            
            else :
                person=tracker_people()
                
            self.person_pub.publish(person)
                    
            rospy.sleep(3)
        
        
if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.loginfo("Initializing follow_me_srv")
    person=Follow_me_Service()
   
    person.run()
    
    
  
