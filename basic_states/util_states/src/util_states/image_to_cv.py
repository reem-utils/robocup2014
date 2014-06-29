#!/usr/bin/env python

"""
@date: June 17 2014
@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com
"""
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = "/stereo/left/image" #It has to be changed to xtion!
cv_image_final = None
PKG_PATH = roslib.packages.get_pkg_dir('emergency_situation')
global image_saved

class image_converter:
    def __init__(self):
        
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.im_saved = False
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC,Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print (cv_image)
            if self.im_saved == False:
                self.im_saved = True
                cv2.imwrite(PKG_PATH+"/config/camera_image.png", cv_image)
        except CvBridgeError, e:
            print e
        #Unregistering from the TOPIC
        self.image_sub.unregister()
         
#         (rows,cols,channels) = cv_image.shape
#         if cols > 60 and rows > 60 :
#             cv2.circle(cv_image, (50,50), 10, 255)
#         
        
        
        #cv2.imshow("Image window", cv_image)
        #cv2.imwrite(PKG_PATH+"/config/camera_image.pgm", cv_image)
        #cv2.waitKey(3)
        
            
#         try:
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#         except CvBridgeError, e:
#             print e

def main(args):
    rospy.init_node('image_converter_node')
    
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    
    #cv2.imwrite(PKG_PATH+"/config/camera_image.pgm", cv_image_final)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
    
