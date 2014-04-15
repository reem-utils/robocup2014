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
MINDIST=0.30
SPEED_X=-0.3 # that is a back speed



MAXIM_INIT=200 # it'a a initialitzation value, it have to be bigger than ultraSounds Range
NUM_MOSTRES=3
'''
@this is a navigation
@The maximum value of Distance is 3 meters
@It have a time out of 15 seconds
@Becareful!! this don't have navigation control!!
@It loock the 3 back ultrasound and control if the distance is les than 30cm (is not a fast detection)
'''
class navigation_back():
    
    def __init__(self):
        rospy.loginfo("Initializing reverse")
        
        self.nav_pub= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        self.nav_srv = rospy.Service('/reverse',NavigationGoBack, self.nav_back_srv)
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
        
        # ulta sound variables
        self.sonar=3*[NUM_MOSTRES*[MAXIM_INIT]]
        self.num_sonar=3*[0]
        self.resultat=3*[MAXIM_INIT]
        
        self.impacte=False
        self.time_out=False
        
        
    def nav_back_srv(self,req):
        
        if (req.meters<=MAXMETERS) :
            if (req.enable) :
               
                self.enable=True
                self.meters=req.meters#number of meters that we have to do
                self.Odometry_init=self.Odometry_actual
                self.time_init= rospy.get_rostime()
                self.ultra_subs=rospy.Subscriber("/sonar_base", Range, self.callback_Sonar)   
                self.run()
            else :
                self.enable=False
                self.pub_stop()
            
            
            if (self.time_out or self.impacte):
                rospy.loginfo("ABORTING!!!")
                if self.time_out :
                    rospy.loginfo("TIME OUT")
                if self.impacte :
                    rospy.loginfo("IMPACT")
                return "ABORTED"
            else :
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
            self.movment=False
        else :
            self.movment=True
            
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
        
        if (data.header.frame_id==(('base_sonar_07_link' ) or ('/base_sonar_07_link' ))):
                self.sonar[0][self.num_sonar[0]]=data.range
                self.num_sonar[0]=self.num_sonar[0]+1
                aux=sum(self.sonar[0])
                self.resultat[0]=aux/NUM_MOSTRES
                if self.num_sonar[0]>=NUM_MOSTRES :
                    self.num_sonar[0]=0
                
        elif (data.header.frame_id==(('base_sonar_08_link')or ('/base_sonar_08_link' ))):
                self.sonar[1][self.num_sonar[1]]=data.range
                self.num_sonar[1]=self.num_sonar[1]+1
                aux=sum(self.sonar[1])
                self.resultat[1]=aux/NUM_MOSTRES
                if self.num_sonar[1]>=NUM_MOSTRES :
                    self.num_sonar[1]=0

            
        elif (data.header.frame_id==(('base_sonar_09_link')or ('/base_sonar_09_link' ))):
                self.sonar[2][self.num_sonar[2]]=data.range
                self.num_sonar[2]=self.num_sonar[2]+1
                aux=sum(self.sonar[2])
                self.resultat[2]=aux/NUM_MOSTRES
                if self.num_sonar[2]>=NUM_MOSTRES :
                    self.num_sonar[2] =0              
        
    def proces_ultraSound(self):
        
        
        if self.resultat[0]<MINDIST or self.resultat[1]<MINDIST or self.resultat[2]<MINDIST :
            rospy.loginfo(str(self.resultat))
            rospy.loginfo(str(self.sonar))
            self.impacte= True
        else :
            self.impacte= False
            
            
    def proces_time(self):
        time_actual=rospy.get_rostime()
        time=time_actual.secs-self.time_init.secs
        
        if time<MAXTIME :
            self.time_out = False
            
        else :
            self.time_out = True
        
        
            
    def run(self):
        
        while not rospy.is_shutdown() and self.enable:
           
            if self.enable: 
                self.proces_odometry()
                self.proces_ultraSound()
                self.proces_time()
                if not self.movment and not self.time_out and not self.impacte :
                    self.pub_move()
                else : 
                    self.pub_stop()
                    self.enable=False
                    self.ultra_subs.unregister()
                    
                rospy.sleep(0.01)
            else :
                rospy.sleep(3)
    def bucle(self):
        while not rospy.is_shutdown():
            rospy.sleep(3)
        
        
        
if __name__ == '__main__':
    rospy.init_node('navigation_reverse_service')
    rospy.sleep(1)
    rospy.loginfo("navigation_reverse_srv")
    navigation = navigation_back()
    navigation.bucle()
    
    
  