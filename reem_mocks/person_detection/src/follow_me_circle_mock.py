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
        
     
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        self.init_time=rospy.get_time()
        while not rospy.is_shutdown(): 
            rospy.loginfo("rospy.Time.now().secs - self.init_time.secs = " +
                          str(rospy.get_time())  + " - " + str(self.init_time)
                          + " = " + str(rospy.get_time() - self.init_time))
            if (rospy.get_time() - self.init_time < 60):
                person=tracker_people()
                person.confidence=100
                person.pose.position.x=0.7
                person.pose.position.y=0.5
                person.pose.orientation.w=1.0
                rospy.loginfo(OKGREEN+"im sending:     "+str(person)+ENDC)
                self.person_pub.publish(person)
                
            else :
                person=tracker_people()
                person.confidence=100
                person.pose.position.x=0.2
                person.pose.position.y=0.0
                person.pose.orientation.w=1.0
                rospy.loginfo(OKGREEN+"im sending:     "+str(person)+ENDC)
                self.person_pub.publish(person)
                
            rospy.sleep(3)
                


if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.sleep(1)
    rospy.loginfo("Initializing follow_me_srv")
    person=Follow_me_Service()
    person.run()
    
    
  
