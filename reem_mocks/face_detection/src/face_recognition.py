#! /usr/bin/env python

import rospy
import actionlib

from pal_detection_msgs.srv import Recognizer
from pal_detection_msgs.srv import RecognizerRequest, RecognizerResponse
from pal_detection_msgs.msg import FaceDetection, FaceDetections
from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest,  StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse
from pal_detection_msgs.srv import SetDatabase, SetDatabaseRequest, SetDatabaseResponse




class FaceService():
    """Face recognition Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing face_recognition_server")
        self.minConfidence = 0.0
        self.enabled = False
        self.name=""
        self.time_first_face=60
        self.time_Start=rospy.Time.now()
        self.recognized_face = FaceDetections()
        self.faceservice = rospy.Service('/pal_face/recognizer', Recognizer, self.face_cb)
        self.faceservice = rospy.Service('/pal_face/start_enrollment',
                                          StartEnrollment, self.face_enrollment_start_cb)
        self.faceservice = rospy.Service('/pal_face/stop_enrollment', 
                                         StopEnrollment, self.face_enrollment_stop_cb)
        self.faceservice = rospy.Service('/pal_face/set_database', 
                                         SetDatabase, self.face_database_cb)
        rospy.loginfo("face service initialized")
        self.face_pub= rospy.Publisher('/pal_face/recognizer', FaceDetections)
       

   
   
        
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
            cara = FaceDetection()
            cara.name = self.name
            cara.confidence = self.minConfidence
            self.recognized_face.faces.append(cara)
        else:
            stop.error_msg="no name face"
            stop.enrollment_ok=False
            stop.numFacesEnrolled=0
         
        return stop
    
    def face_database_cb (self, req):
        
        """Callback of face service requests """
    #        rospy.loginfo("FACE: Disabling face enrollment" )
        database = SetDatabaseResponse()
        #self = SetDatabaseRequest()
        # it means that delates the databas
        if req.purgeAll==True :
            self.recognized_face = FaceDetections()
            rospy.loginfo("removing database  " + str(req.databaseName) )
        else:
            rospy.loginfo("Charging database  "+str(req.databaseName) )
        return database # todo: i dont know what 
    def asck_mode(self):
        self.time_first_face=str(raw_input('Time for start introducing face :'))    
        self.time_Start=rospy.Time.now()
         
    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled:
                #rospy.loginfo("FACE: Disabling face recognition" )
                self.recognized_face.header.stamp=rospy.Time.now()
                self.recognized_face.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                if ((self.recognized_face.header.stamp.secs-self.time_Start.secs)>int(self.time_first_face) ):
                    for face in self.recognized_face.faces:
                        face.x= 262
                        face.y= 200
                        face.width= 61
                        face.height= 61
                        face.eyesLocated= True
                        face.leftEyeX= 307
                        face.leftEyeY= 215
                        face.rightEyeX= 276
                        face.rightEyeY= 217
                        face.position.x=-0.0874395146966
                        face.position.y= -0.0155512560159
                        face.position.z= 0.945071995258
                    self.face_pub.publish(self.recognized_face)
                
                else:
                    pub=FaceDetections()
                    pub.header.stamp=rospy.Time.now()
                    pub.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                    self.face_pub.publish(pub)
                
            rospy.sleep(3)
        
        
if __name__ == '__main__':
    rospy.init_node('face_recognizer_srv')
    rospy.loginfo("Initializing face_recognizer_srv")
    face = FaceService()
    face.asck_mode()
    face.run()
    
    
  

# vim: expandtab ts=4 sw=4
