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
from pal_detection_msgs.msg import FaceDetections,Detections2d,FaceDetection,Detection2d


fakepositionx=0
fakepositiony=2
fakeorientation=0
information=True
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
from smach_ros import ServiceState

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'




'''
@It publish a message similar from the face detection, this one have de doble heigh
it will publish a Detections2d msg
in the /gesture_recognition/face_square'


'''





class Follow_me_Service():
    """
    Follow me Mock 

    Run the topic of face recognition:
        /people_tracker/person

    """
    def face_cb(self,data):
        faces=data
        Face=FaceDetection()
        for Face in faces.faces :
            if(Face.name==self.name):
                person=tracker_people()
                person.confidence=Face.confidence
                person.pose.position=Face.position
                
        self.person_pub.publish(person)
                
    def __init__(self):
        self.name=str(raw_input('Name of the face :'))  
        rospy.loginfo("Initializing tracker_people_service")
        self.person_pub=rospy.Publisher('/people_tracker/person',tracker_people)
        self.face_subs=rospy.Subscriber("/pal_face/recognizer", FaceDetections, self.face_cb)
     
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown(): 
                rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.loginfo("Initializing follow_me_srv")
    person=Follow_me_Service()
    
    person.run()
    
    
  
