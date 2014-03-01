#! /usr/bin/env python

import rospy
import actionlib

from pal_detection_msgs.srv import Recognizer
from pal_detection_msgs.srv import RecognizerRequest, RecognizerResponse
from pal_detection_msgs.msg import FaceDetection



class FaceService():
    """Face recognition Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing face_service")
        self.minConfidence = 0.0
        self.enabled = False
        self.faceservice = rospy.Service('/faceservice', Recognizer, self.face_cb)
        rospy.loginfo("face service initialized")
        self.face_pub= rospy.Publisher('/facedetection', FaceDetection)
        
    def face_cb(self, req):
        """Callback of face service requests """
        if req.enabled:
            rospy.loginfo("FACE: Enabling face recognition '%s'" % req.enabled)
            self.minConfidence = req.minConfidence
            self.enabled = True
        else:
            rospy.loginfo("FACE: Disabling face recognition" )
            self.minConfidence = 0.0
            self.enabled = False       
        return RecognizerResponse()
    
        
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled:
                recognized_face = FaceDetection()
                recognized_face.name = "Pepito"
                recognized_face.confidence = 50.0
                self.face_pub.publish(recognized_face)
            rospy.sleep(1)
        
        
                
if __name__ == '__main__':
  rospy.init_node('face_srv')
  rospy.loginfo("Initializing face_srv")
  face = FaceService()
  face.run()
  

# vim: expandtab ts=4 sw=4
