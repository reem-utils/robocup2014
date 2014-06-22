#! /usr/bin/env python
'''
@author: Roger Boldu
'''


import rospy
import actionlib
import sys
import select


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
FRONT_DISTANCE=0.5
RIGTH_DISTANCE=0.5
LEFT_DISTAMCE=0.5
BACK_DISTANCE=0.4
min_distance_Laser = 1
TIME_POSE=0.1
MIN_DISTANCE=0.05


from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from util_states.topic_reader import topic_reader
from rospy.core import rospyinfo
from util_states.math_utils import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from util_states.topic_reader import topic_reader
from bzrlib import switch
from nose import case
from navigation_states.get_current_robot_pose import get_current_robot_pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from follow_me.msg import check_elevator
from follow_me.srv import EnableCheckElevator
from follow_me.srv import EnableCheckElevatorRequest, EnableCheckElevatorResponse


'''
@it have an enable to start the publish on /check_elevator/elevator_status
@ This enable it have to be started whith the service /check_elevator/enable


'''
class checkElevator():
    
    def __init__(self):
        rospy.loginfo("Initializing check_elevator")
        
        self.elevator_pub= rospy.Publisher('/check_elevator/elevator_status', check_elevator)
        self.elevator = rospy.Service('/check_elevator/enable',EnableCheckElevator, self.elevator_srv)

        self.init_var()
    def elevator_srv(self,req):
        response = EnableCheckElevatorResponse()
        if (req.enable) :
            self.enable=True
            self.sonar_sub=rospy.Subscriber("/sonar_base", Range, self.callback_Sonar)
            self.filter_sub=rospy.Subscriber("/scan_filtered", LaserScan, self.callback_Laser)
            self.pose_sub=rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.callback_Pose)
            #self.init_var()
        else :
            self.pose_sub.unregister()
            self.sonar_sub.unregister()
            self.filter_sub.unregister()
            self.enable=False
        
        return EnableCheckElevatorResponse()
            # TODO: it can be interesting to not subscriber the topics
    def init_var(self):
        self.enable=False
        self.ultraSound=[]
        aux = Range()
        self.ultraSound.append(aux)
        self.ultraSound.append(aux)
        self.ultraSound.append(aux)
        self.ultraSound.append(aux)
        aux=False
        self.ultra=[]
        self.ultra.append(aux)
        self.ultra.append(aux)
        self.ultra.append(aux)
        self.ultra.append(aux) 
        self.ultra_sound_ready=False
        self.ultra_status=False
        self.ultra_door=False
        
        
        self.Laser_status=0
        
        
        self.position=None
        self.position_status=0
        
    def callback_Sonar(self,data):
        
        if (data.header.frame_id=='/base_sonar_02_link' or data.header.frame_id=='base_sonar_02_link'):
                            self.ultraSound[0]=data
                            self.ultra[0]=True

        
        if (data.header.frame_id=='/base_sonar_05_link' or data.header.frame_id=='base_sonar_05_link'):
                            self.ultraSound[1]=data
                            self.ultra[1]=True
 
         
        if (data.header.frame_id=='/base_sonar_08_link' or data.header.frame_id=='base_sonar_08_link'):
                            self.ultraSound[2]=data
                            self.ultra[2]=True
        
        if (data.header.frame_id=='/base_sonar_11_link' or data.header.frame_id=='base_sonar_11_link'):
                            self.ultraSound[3]=data
                            self.ultra[3]=True 
        
        if (self.ultra[0] and self.ultra[1] and self.ultra[2] and self.ultra[3]) :
            self.ultra_sound_ready=True
        else :
            self.ultra_sound_ready=False    
                                                                                                      
    def callback_Laser(self,data):
        
        
        length=len(data.ranges)  
        
        if data.ranges[1]<min_distance_Laser and data.ranges[length-1]<min_distance_Laser :
            self.Laser_status=True
        else :
            self.Laser_status=False
        

    def callback_Pose(self,data):  
        self.position=data

    def proces_position(self):
        
        if self.position!=None :
            position1=self.position
            rospy.sleep(TIME_POSE)
            position2=self.position
            if (position1.pose.pose.position.x+MIN_DISTANCE)>position2.pose.pose.position.x and (position1.pose.pose.position.x-MIN_DISTANCE)<position2.pose.pose.position.x:
            
                if (position1.pose.pose.position.y+MIN_DISTANCE)>position2.pose.pose.position.y and (position1.pose.pose.position.y-MIN_DISTANCE)<position2.pose.pose.position.y :
                    self.position_status=True
                else :
                    self.position_status=False
            else :
                self.position_status=False
        else :
            self.position_status=False      
    
    def proces_ultra_sound(self):
        rospy.loginfo(OKGREEN+str(self.ultraSound)+ENDC)
        self.nUltra=0
        if (not self.ultra_sound_ready):
            self.ultra_status=False
        else :
           
            if self.ultraSound[0].range<FRONT_DISTANCE :
                self.nUltra=self.nUltra+1
                # Left
            if self.ultraSound[1].range<LEFT_DISTAMCE :
                self.nUltra=self.nUltra+1
                #Back
            if self.ultraSound[2].range<BACK_DISTANCE :
                self.nUltra=self.nUltra+1
                self.ultra_door=True
            else :
                self.ultra_door=False
                #Rigth
            if self.ultraSound[3].range<RIGTH_DISTANCE :
                self.nUltra=self.nUltra+1
            
            if self.nUltra==4 :
                self.ultra_status=True
            else :
                self.ultra_status=False
                #rospy.loginfo(OKGREEN+str(self.ultraSound)+ENDC)
                
                        
                
            
         
    def run(self):
        msg=check_elevator()

        while not rospy.is_shutdown():
            if self.enable :
                self.proces_position()
                self.proces_ultra_sound()
                msg.laser=self.Laser_status # the value it's 1 if the value of it's extrems is less than the minimum
                msg.ultra_sound=self.ultra_status # the value is 1 if all the ultra sound are " in the minimun distance"
                msg.ultra_sound_door=self.ultra_door # i put if the dor it's open or close
                msg.moving=self.position_status
                if msg.laser and msg.ultra_sound and msg.moving :# no elevator
                    msg.elevator=True
                else :
                    msg.elevator=0
                #rospy.loginfo(OKGREEN)  
                #rospy.loginfo("\n"+str(msg)+ENDC)  
                self.elevator_pub.publish(msg)    
            rospy.sleep(0.5)
        
        
        
        
if __name__ == '__main__':
    rospy.init_node('check_elevator_service')
    rospy.loginfo("check_elevator_srv")
    elevator = checkElevator()
    elevator.run()
    
    
  