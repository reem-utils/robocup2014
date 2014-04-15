
import smach
import rospy
import rosparam
import roslib

from emergency_situation.image_creator import ImageCreator
from emergency_situation.pdf_creator import create_pdf
import os
import shutil

# Loading the parameters depending on which robot we are using.
# Reduce the size of image file

robot = os.environ.get('PAL_ROBOT')
if robot == 'rh2c' or robot == 'rh2m':
    IMAGE_PATH = roslib.packages.get_pkg_dir('reemh2_maps') + '/config/'
    RH2_PATH = roslib.packages.get_pkg_dir('reemh2_maps')
    paramlist = rosparam.load_file(RH2_PATH+"/config/map.yaml", default_namespace="emergency_situation")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

elif robot == 'reemh3c' or robot == 'reemh3m':
    IMAGE_PATH = roslib.packages.get_pkg_dir('reem_maps') + '/config/'
    REEMH3_PATH = roslib.packages.get_pkg_dir('reem_maps')
    paramlist = rosparam.load_file(REEMH3_PATH+'/config/map.yaml', default_namespace="emergency_situation")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

else:
    #IMAGE_PATH = roslib.packages.get_pkg_dir('robocup_iworlds') + '/navigation/'
    IMAGE_PATH = roslib.packages.get_pkg_dir('emergency_situation') + '/config/'
    #ROBOCUP_PATH = roslib.packages.get_pkg_dir('robocup_worlds')
    #paramlist = rosparam.load_file(ROBOCUP_PATH+"/navigation/subMap1.yaml", default_namespace="emergency_situation")
    paramlist = rosparam.load_file(IMAGE_PATH+"/subMap1.yaml",default_namespace="emergency_situation")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

PKG_PATH = roslib.packages.get_pkg_dir('emergency_situation')
RESOLUTION = rospy.get_param("/emergency_situation/resolution")
IMAGE_NAME = rospy.get_param("/emergency_situation/image")
IMAGE_ORIGIN = rospy.get_param("/emergency_situation/origin")
IMAGE_ORIGIN = [0, 0]
IMAGE_NAME = "subMap_1.pgm"


class GeneratePDF_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                                    input_keys=['person_location'])
        self.pendrive_location = rospy.get_param('/emergency_situation/pendrive_location')
        print "\nPendrive Location: %s\n" % self.pendrive_location

    def execute(self, userdata):
        rospy.loginfo("Informing Ambulance...............")
        image_created_number = ImageCreator(location_list=[userdata.person_location.pose.pose.position.x, userdata.person_location.pose.pose.position.y], 
                                        origin=IMAGE_ORIGIN,
                                        scale=RESOLUTION, 
                                        image_name=IMAGE_NAME, 
                                        pkg_path=PKG_PATH, 
                                        image_path=IMAGE_PATH)
        create_pdf(PKG_PATH + '/config/')
        print "PDF CREATED in  " + PKG_PATH + '/config/reem3.pdf'
        rospy.loginfo("Copying file to %s/reem3.pdf" % self.pendrive_location)
        #shutil.copy(PKG_PATH + '/config/' + "reem3.pdf", self.pendrive_location + '/reem3.pdf')
        rospy.loginfo("Ambulance informed successfully !!!!")
        return 'succeeded'







