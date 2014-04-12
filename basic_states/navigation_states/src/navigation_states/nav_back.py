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

from geometry_msgs.msg import Twist
from get_current_robot_pose import get_current_robot_pose
from navigation_states.srv import NavigationGoBack,NavigationGoBackResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from util_states.math_utils import *
from sensor_msgs.msg import Range # that is the ultrasound msgs

MAXMETERS=3 # this is the maximum numer of meters that can move the robot back
MAXTIME=15 # numer maxim of time that the robot can be going back
MINTDIST=0.30
IMPACT=100 # every back sensor that give a impact result increments INC, and the no impact decrement DEC
INC=40 #every detection it will increment 
DEC=10 # no detections will decrement 
SPEED_X=-0.3 # that is a back speed
'''
@this is a navigation
@The maximum value of Distance is 3 meters
@It have a time out of 15 seconds
@Becareful!! this don't have navigation control!!
@It loock the 3 back ultrasound and control if the distance is les than 30cm (is not a fast detection)
'''
class navigation_back():
    
    def __init__(self):
        rospy.loginfo("Initializing navigation_back")
        
        self.nav_pub= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        self.nav_srv = rospy.Service('/move_back',NavigationGoBack, self.nav_back_srv)
        self.odom_subs = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.check_Odometry)
        
        #self.odom_subs.unregister()
        self.init_var()
        
    def init_var(self):
        self.Odometry_actual=Odometry()
        self.Odometry_init=None
        self.enable=False
        self.time_init= rospy.get_rostime()
        self.impacte=0
        self.goal_achieved=False
        
    def nav_back_srv(self,req):
        
        if (req.meters<=MAXMETERS) :
            if (req.enable) :
                self.enable=True
                self.meters=req.meters#number of meters that we have to do
                self.Odometry_init=self.Odometry_actual
                self.time_init= rospy.get_rostime()
                self.ultra_subs=rospy.Subscriber("/sonar_base", Range, self.callback_Sonar)
                self.impacte=0
                self.goal_achieved = False
            else :
                
                self.enable=False
                self.pub_stop()
            
            while not self.goal_achieved :
                rospy.sleep(0.1)
            
            return NavigationGoBackResponse()
        else :
            self.pub_stop()
            return "To much distance"
        
    def check_Odometry(self,data):
        self.Odometry_actual=data
        
    def proces_odometry(self):
        position_navigate=Pose()
        position_navigate.position.x=self.Odometry_actual.pose.pose.position.x-self.Odometry_init.pose.pose.position.x
        position_navigate.position.y=self.Odometry_actual.pose.pose.position.y-self.Odometry_init.pose.pose.position.y
        
        unit_vector = normalize_vector(position_navigate.position)
        position_distance = vector_magnitude(position_navigate.position)
        if (position_distance<self.meters) :
            self.movment=True
        else :
            self.movment=False
    def pub_move(self):
        msg=Twist()
        msg.linear.x= SPEED_X
        msg.linear.y=0
        msg.linear.z=0
        msg.angular.x=0
        msg.angular.y=0
        msg.angular.z=0
        self.nav_pub.publish(msg)
        
    def pub_stop(self):
        msg=Twist()
        msg.linear.x=0
        msg.linear.y=0
        msg.linear.z=0
        msg.angular.x=0
        msg.angular.y=0
        msg.angular.z=0

        self.nav_pub.publish(msg)
        
    def callback_Sonar(self,data):
        
        if (data.header.frame_id==('/base_sonar_07_link' )):
            if data.range<MINTDIST :
                self.impacte=self.impacte+INC
                
            else :
                self.impacte=self.impacte-DEC
               
                if self.impacte<0 :
                    self.impacte=0
        elif (data.header.frame_id==('/base_sonar_08_link')):
            if data.range<MINTDIST :
                self.impacte=self.impacte+INC
               
            else :
                self.impacte=self.impacte-DEC
               
                if self.impacte<0 :
                    self.impacte=0
            
        elif (data.header.frame_id==('/base_sonar_09_link')):
            if data.range<MINTDIST :
                self.impacte=self.impacte+INC
                
            else :
                
                self.impacte=self.impacte-DEC
                
                if self.impacte<0 :
                    self.impacte=0
        
    
    def run(self):
        msg=Twist()
        
        while not rospy.is_shutdown():
            time_actual=rospy.get_rostime()
            
            time=time_actual.secs-self.time_init.secs
            if self.enable: 
                self.proces_odometry()
                if self.movment and time<MAXTIME and self.impacte<IMPACT :
                    self.pub_move()
                else :
                    if time>MAXTIME :
                        rospy.loginfo(FAIL+"TIME OUT MOVE_BACK"+ENDC)
                    if  self.impacte>=IMPACT :
                        rospy.loginfo(FAIL+"IMPACT"+ENDC)
                    self.goal_achieved=True # TODO: this is complicate
                    self.pub_stop()
                    self.enable=False
                    self.ultra_subs.unregister()
                    
                rospy.sleep(0.01)
            else :
                rospy.sleep(3)
        
        
        
        
if __name__ == '__main__':
    rospy.init_node('navigation_back_service')
    rospy.sleep(1)
    rospy.loginfo("navigation_back_srv")
    navigation = navigation_back()
    navigation.run()
    
    
  