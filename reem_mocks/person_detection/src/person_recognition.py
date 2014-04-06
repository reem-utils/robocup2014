#! /usr/bin/env python

'''
Created on 16/03/2014

@author: Cristina De Saint Germain
'''

import rospy
import actionlib
import select
import tty
import sys

from pal_detection_msgs.msg import PersonDetection, PersonDetections
from pal_detection_msgs.msg import Detection2d, FaceDetection, LegDetections
from geometry_msgs.msg import Point

class PersonTopic():
    """
	Person detection Mock 

	Run the topic of face recognition:
		/pal_person/recognizer
 	"""
    def __init__(self):
        rospy.loginfo("Initializing person_recognizer topic")
        self.person_pub= rospy.Publisher('/pal_person/recognizer', PersonDetections) 
        self.person_recognizer = PersonDetections()  

    def isData(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    def random_person(self):
        person = PersonDetection()
                
        # New body
        person.full_body.x = 1.0
        person.full_body.y = 1.0
        person.full_body.width = 5.0
        person.full_body.height = 5.0
        
        # New face
        person.face.x= 262
        person.face.y= 200
        person.face.width= 61
        person.face.height= 61
        person.face.eyesLocated= True
        person.face.leftEyeX= 307
        person.face.leftEyeY= 215
        person.face.rightEyeX= 276
        person.face.rightEyeY= 217
        person.face.position.x=-0.0874395146966
        person.face.position.y= -0.0155512560159
        person.face.position.z= 0.945071995258
        
        # New legs
        leg = Point()
        leg.x = 1.0
        leg.y = 1.0
        leg.z = 0.0
        person.legs.position3D.append(leg)
        
        return person
        
    def run(self):
        
        while not rospy.is_shutdown():
            
            if self.isData():
                
                c = sys.stdin.read(1) #Clear buffer
                
                # Erase the database
                if c == 'r':
                    self.person_recognizer = PersonDetections()  
                else:
                    # Create a new person
                    self.person_recognizer.persons.append(self.random_person())          
                
            self.person_recognizer.header.stamp=rospy.Time.now()
            # Show the info   
            self.person_pub.publish(self.person_recognizer)
           
            
            rospy.sleep(3)
        
        
if __name__ == '__main__':
    rospy.init_node('person_recognizer_srv')
    rospy.loginfo("Initializing person_recognizer_srv")
    face = PersonTopic()
    face.run()
