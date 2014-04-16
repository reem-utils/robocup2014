#! /usr/bin/env python

import rospy
import actionlib
import select
import sys

from grasping_mock.msg import ObjectDetections, ObjectDetection
from grasping_mock.srv import Recognizer, RecognizerResponse, RecognizerRequest

class ObjectService():
    """
    This is a Service for Object Recognition.

    It publishes the messages (of type 'ObjectDetection') to the topic '/object_detect/recognizer'.
    """
    
    def __init__(self):
        rospy.loginfo("Initializing object_recognition_server")
        self.minConfidence = 0.0
        self.object_id = 1
        self.enabled = False
        self.name=""
        self.recognized_object = ObjectDetections()

        self.objectservice = rospy.Service('/object_detect/recognizer', Recognizer, self.object_cb)
        self.object_pub= rospy.Publisher('/object_detect/recognizer', ObjectDetections)

    def object_cb(self, req):
        """Callback of object service requests """
        if (req.enabled):
            self.minConfidence = req.minConfidence
            self.enabled = True
        else:
            self.minConfidence = 0.0
            self.enabled = False
        return RecognizerResponse()

    def isData(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) 

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

                if self.isData():
    
                    name = sys.stdin.readline() #Clear buffer

                    objectm = ObjectDetection()
                    objectm.object_id = self.object_id
                    objectm.object_name.data = name[:len(name)-1:]
                    objectm.position.x=-0.0874395146966
                    objectm.position.y= -0.0155512560159
                    objectm.position.z= 0.945071995258
                    self.object_id =+ 1 
                    
                    self.recognized_object.objects.append(objectm)

                self.object_pub.publish(self.recognized_object)
                
            rospy.sleep(3)

        
if __name__ == '__main__':
    rospy.init_node('object_recognizer_srv')
    rospy.loginfo("Initializing object_recognizer_srv")
    object_d = ObjectService()
    object_d.run()
   

# vim: expandtab ts=4 sw=4
