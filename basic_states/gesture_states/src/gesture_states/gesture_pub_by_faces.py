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

from pal_detection_msgs.msg import FaceDetections,Detections2d,FaceDetection,Detection2d



'''
@It publish a message similar from the face detection, this one have de doble heigh
it will publish a Detections2d msg
in the /gesture_recognition/face_square'


'''
class gesture_creat():
    
    def __init__(self):
        rospy.loginfo("Initializing gesture_by_face")
        self.elevator_pub= rospy.Publisher('/gesture_recognition/face_square', Detections2d)
        self.init_var()
        self.face_subs=rospy.Subscriber("/pal_face/recognizer", FaceDetections, self.face_cb)
           

    def init_var(self):

        
        
        self.Laser_status=0
        
        
        self.position=None
        self.position_status=0
        
    def face_cb(self,data):
        faces=data
        squares=Detections2d()
        i=0 
        for Face in faces.faces :
                
            square=Detection2d()
            square.x=Face.x
            square.y=Face.y

            square.width=Face.width
            square.height=Face.width*2
            squares.detections.append(square)
            rospy.loginfo(OKGREEN+str(square)+ENDC)
            
        self.elevator_pub.publish(squares)
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)
        
        
        
        
if __name__ == '__main__':
    rospy.init_node('pub_gesture_from_face')
    rospy.loginfo("pub_gesture_from_face_service")
    gesture = gesture_creat()
    gesture.run()
    
    
