#! /usr/bin/env python

import rospy
import actionlib

from grasping_mock.msgs import ObjectDetection
from grasping_mock.srv import Recognizer, RecognizerResponse, RecognizerRequest

class ObjectService():
    """Face recognition Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing face_recognition_server")
        self.minConfidence = 0.0
        self.enabled = False
        self.name=""
        self.time_first_face=60
        self.time_Start=rospy.Time.now()
        self.recognized_object = ObjectDetection()

        self.faceservice = rospy.Service('/object_detect/recognizer', ObjectDetection, self.face_cb)
        self.face_pub= rospy.Publisher('/object_detect/recognizer', ObjectDetection)

    def object_cb(self):
        """Callback of object service requests """
        if (req.enabled):
            self.minConfidence = req.minConfidence
            self.enabled = True
        else:
            self.minConfidence = 0.0
            self.enabled = False
        return RecognizerResponse()

    def asck_mode(self):
        self.time_first_object=str(raw_input('Time for start introducing object :'))    
        self.time_Start=rospy.Time.now()

    def run(self):
        """Publishing usersaid when face recognitionL is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled:
                #rospy.loginfo("FACE: Disabling face recognition" )
                self.recognized_object.header.stamp=rospy.Time.now()
                self.recognized_object.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                if ((self.recognized_object.header.stamp.secs-self.time_Start.secs)>int(self.time_first_object) ):
                    face.position.x=-0.0874395146966
                    face.position.y= -0.0155512560159
                    face.position.z= 0.945071995258
                    self.face_pub.publish(self.recognized_object)
                
                else:
                    pub=FaceDetections()
                    pub.header.stamp=rospy.Time.now()
                    pub.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                    self.face_pub.publish(pub)
                
            rospy.sleep(3)
        
        
if __name__ == '__main__':
    rospy.init_node('object_recognizer_srv')
    rospy.loginfo("Initializing object_recognizer_srv")
    object_d = ObjectService()
    object_d.asck_mode()
    object_d.run()
    
    
  

# vim: expandtab ts=4 sw=4