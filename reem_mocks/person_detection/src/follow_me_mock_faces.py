#! /usr/bin/env python
'''
@author: Roger Boldu
'''

import smach
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
from follow_me.msg import tracker_people,person,personArray
from pal_detection_msgs.msg import FaceDetections,Detections2d,FaceDetection,Detection2d
from smach_ros import ServiceState

fakepositionx=0
fakepositiony=2
fakeorientation=0
information=True
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
        /people_tracker_node/peopleSet

    """
    def face_cb(self,data):
        faces=data
        Face=FaceDetection()
        per=person()
        
        if faces.faces :
            for Face in faces.faces :
                per.targetId=1
                per.x=Face.position.z
                per.y=-Face.position.x
                per.status=4
                self.persons.peopleSet.append(per)
            self.follow_person_pub.publish(self.persons)
                
    def __init__(self):
        self.name=str(raw_input('Name of the face :'))  
        rospy.loginfo("Initializing tracker_people_service")
        self.follow_person_pub=rospy.Publisher('/people_tracker_node/peopleSet',personArray)
        self.face_subs=rospy.Subscriber("/pal_face/faces", FaceDetections, self.face_cb)
        self.persons=personArray()
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown(): 
                rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.loginfo("Initializing follow_me_srv")
    follow_persn=Follow_me_Service()
    
    follow_persn.run()
    
    
  
