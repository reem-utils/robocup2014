#! /usr/bin/env python

import rospy
import actionlib

from pal_detection_msgs.srv import Recognizer
from pal_detection_msgs.srv import RecognizerRequest, RecognizerResponse
from pal_detection_msgs.msg import FaceDetection
from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest,  StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse




class FaceService():
    """Face recognition Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing face_recognition_server")
        self.minConfidence = 0.0
        self.enabled = False
        self.name=""
        self.faceservice = rospy.Service('/pal_face/recognizer', Recognizer, self.face_cb)
        self.faceservice = rospy.Service('/pal_face/start_enrollment',
                                          StartEnrollment, self.face_enrollment_start_cb)
        self.faceservice = rospy.Service('/pal_face/stop_enrollment', 
                                         StopEnrollment, self.face_enrollment_stop_cb)
        rospy.loginfo("face service initialized")
        self.face_pub= rospy.Publisher('/pal_face/recognizer', FaceDetection)
   
   
        
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
    
    
    
    def face_enrollment_start_cb(self, req):
        """Callback of face service requests """
        response=StartEnrollmentResponse()
        if req.name!="":
            rospy.loginfo("FACE: Enabling face start_enrollment")
            print str(req.name)
            self.name=req.name  
            response.result=True  
        else:
            rospy.loginfo("FACE: No face name")
            response.result=False
            
        return response
    
    def face_enrollment_stop_cb (self, req):
        
        """Callback of face service requests """
    #        rospy.loginfo("FACE: Disabling face enrollment" )
        stop = StopEnrollmentResponse()
        if self.name!="" :
            
            stop.enrollment_ok=True
            stop.numFacesEnrolled=1
            stop.error_msg=''
            self.name=""
        else:
            stop.error_msg="no name face"
            stop.enrollment_ok=False
            stop.numFacesEnrolled=0
         
        return stop
         
         
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled:
                #rospy.loginfo("FACE: Disabling face recognition" )
                recognized_face = FaceDetection()
                recognized_face.name = "Pepito"
                recognized_face.confidence = 50.0
                self.face_pub.publish(recognized_face)
            rospy.sleep(3)
        
        
if __name__ == '__main__':
    rospy.init_node('face_recognizer_srv')
    rospy.loginfo("Initializing face_recognizer_srv")
    face = FaceService()
    face.run()
  

# vim: expandtab ts=4 sw=4
