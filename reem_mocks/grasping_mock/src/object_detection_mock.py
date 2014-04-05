#! /usr/bin/env python

import rospy
import actionlib

from grasping_mock.msg import ObjectDetection
from grasping_mock.srv import Recognizer, RecognizerResponse, RecognizerRequest

class ObjectService():
    """
    This is a Service for Object Recognition.

    It publishes the messages (of type 'ObjectDetection') to the topic '/object_detect/recognizer'.
    """
    
    def __init__(self):
        rospy.loginfo("Initializing object_recognition_server")
        self.minConfidence = 0.0
        self.enabled = False
        self.name=""
        self.time_first_object=60
        self.time_Start=rospy.Time.now()
        self.recognized_object = ObjectDetection()

        self.objectservice = rospy.Service('/object_detect/recognizer', Recognizer, self.object_cb)
        self.object_pub= rospy.Publisher('/object_detect/recognizer', ObjectDetection)

    def object_cb(self, req):
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
        """
        Publish the position of the object to recognize.
        The Pose has the ID frame as '/stereo_gazebo_right_camera_optical_frame' 
        """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled:
                self.recognized_object.header.stamp=rospy.Time.now()
                self.recognized_object.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                if ((self.recognized_object.header.stamp.secs-self.time_Start.secs)>int(self.time_first_object) ):
                    objectm = ObjectDetection()
		    objectm.position.x=-0.0874395146966
                    objectm.position.y= -0.0155512560159
                    objectm.position.z= 0.945071995258
                    self.object_pub.publish(self.recognized_object)
                
                else:
                    pub=ObjectDetection()
                    pub.header.stamp=rospy.Time.now()
                    pub.header.frame_id = '/stereo_gazebo_right_camera_optical_frame'
                    self.object_pub.publish(pub)
                
            rospy.sleep(3)

        
if __name__ == '__main__':
    rospy.init_node('object_recognizer_srv')
    rospy.loginfo("Initializing object_recognizer_srv")
    object_d = ObjectService()
    object_d.asck_mode()
    object_d.run()
    
    
  

# vim: expandtab ts=4 sw=4
