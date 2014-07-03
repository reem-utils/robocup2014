#! /usr/bin/env python
'''
@author: Roger Boldu
'''

import rospy
import actionlib
import sys
import select

from geometry_msgs.msg import Twist
from get_current_robot_pose import get_current_robot_pose
from navigation_states.srv import NavigationGoTable,NavigationGoTableResponse,NavigationGoTableRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from util_states.math_utils import *
from sensor_msgs.msg import Range # that is the ultrasound msgs
from navigation_states.srv._NavigationGoTable import NavigationGoTableResponse

MAXTIME=30 # numer maxim of time that the robot can be going back
TABLE_DIST=0.75 # to detect de distance of the table
SPEED_X=0.5 # that is a speed if i don't detect nothing
MAXIM_INIT=1.5 # it'a a initialitzation value, it have to be bigger than ir Range

SPEED_LLINDAR=1.3 # here i reduce the speed is the distance of the IR
SPEED_NEAR=0.1
ROBOT=True # it change the publisher
NUM_MOSTRES=20
Topic_IR="ir_base"

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

'''
@this is a navigation table node
@The maximum value of Distance doesn't exist
@It have a time out of 30 seconds
@Becareful!! this don't have navigation control!!
@SPEED: when detects that have a possible table it reduce de speed
@It loock the IR_SENSORS if are 75cm it means that is a table (is not a fast detection)
@It returns succesfull if it found a table
'''

class navigation_back():
    
    def __init__(self):
        rospy.loginfo("Initializing reverse")
        
        if ROBOT :
            self.nav_pub= rospy.Publisher('/key_vel', Twist)
        else :
            self.nav_pub= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        self.nav_srv = rospy.Service('/table_node',NavigationGoTable, self.nav_table_srv)
        self.odom_subs = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.check_Odometry)
        self.init_var()
        
    def init_var(self):
        self.Odometry_actual=Odometry()
        self.Odometry_init=None
        self.enable=False
        
        self.table=0
        self.goal_achieved=False
        self.meters=0
        self.mode=False
        self.time_init= rospy.get_rostime()
        # ulta sound variables
        self.ir = [[MAXIM_INIT for y in xrange(NUM_MOSTRES)] for x in xrange(3)]
        self.num_ir=3*[0]
        self.resultat=3*[MAXIM_INIT]
        
        self.table=False
        self.time_out=False
        
        
    def nav_table_srv(self,req):
        
        if (req.enable) :
            
            self.table=False
            self.time_out=False
            self.ir=[[MAXIM_INIT for y in xrange(NUM_MOSTRES)] for x in xrange(3)]
            self.num_ir=3*[0]
            self.resultat=3*[MAXIM_INIT]
            self.time_init= rospy.get_rostime()
            self.meters=req.meter
            
            self.ultra_subs=rospy.Subscriber(Topic_IR, Range, self.callback_IR)  
            self.enable=True
            self.Odometry_init=self.Odometry_actual
            
            rospy.sleep(0.5)
            self.run()
        else :
            print "emergenci stop"
            self.enable=False
            self.pub_stop()
        
        
        if (self.time_out or not self.table):
            rospy.loginfo("ABORTING!!!")
            rospy.loginfo(str(self.resultat))
            rospy.loginfo(str(self.ir))
            if self.time_out :
                rospy.loginfo("TIME OUT")
            if self.table :
                rospy.loginfo("IMPACT")
            return "ABORTED"
        else :
            return NavigationGoTableResponse()
        
        
        
    def check_Odometry(self,data):
        self.Odometry_actual=data
        
    def proces_odometry(self):
        print "processing odometry"
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
        if not self.mode : 
            msg.linear.x= SPEED_X
        else :
            msg.linear.x=SPEED_NEAR
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
        
    def callback_IR(self,data):
        
        timestamp = rospy.Time.now()
        if (data.header.frame_id==('/base_ir_03_link') or data.header.frame_id==('base_ir_03_link')):
                self.ir[0][self.num_ir[0]]=data.range
                self.num_ir[0]=self.num_ir[0]+1
                aux=sum(self.ir[0])
                self.resultat[0]=aux/NUM_MOSTRES
                if self.num_ir[0]>=NUM_MOSTRES :
                    self.num_ir[0]=0
                
        elif (data.header.frame_id==('/base_ir_02_link' ) or data.header.frame_id==('base_ir_02_link' )):
                self.ir[1][self.num_ir[1]]=data.range
                self.num_ir[1]=self.num_ir[1]+1
                aux=sum(self.ir[1])
                self.resultat[1]=aux/NUM_MOSTRES
                if self.num_ir[1]>=NUM_MOSTRES :
                    self.num_ir[1]=0

        elif (data.header.frame_id==('/base_ir_01_link' ) or data.header.frame_id==('base_ir_01_link' )):
                self.ir[2][self.num_ir[2]]=data.range
                self.num_ir[2]=self.num_ir[2]+1
                aux=sum(self.ir[2])
                self.resultat[2]=aux/NUM_MOSTRES
                if self.num_ir[2]>=NUM_MOSTRES :
                    self.num_ir[2] =0
        
    def proces_ir(self):    
        print str(self.resultat[0]) + " < " +str (TABLE_DIST) + " or "+ str(self.resultat[1]) + " < " +str(TABLE_DIST) + " or " +str(self.resultat[2]) + " < " +str(TABLE_DIST)
        if self.resultat[0]<TABLE_DIST or self.resultat[1]<TABLE_DIST or self.resultat[2]<TABLE_DIST :
            self.table= True
            self.mode=False
        else :
            if (self.resultat[0]<SPEED_LLINDAR  or self.resultat[1]<SPEED_LLINDAR or self.resultat[2])<SPEED_LLINDAR :
                self.mode=True
            else :
                self.mode=False
            self.table= False
            
            
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
                self.proces_ir()
                self.proces_time()

                if self.movment and not self.time_out and not self.table :
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
    rospy.init_node('navigation_table_service')
    rospy.sleep(1)
    rospy.loginfo("navigation_table_srv")
    navigation = navigation_back()
    navigation.bucle()
    
    
  
