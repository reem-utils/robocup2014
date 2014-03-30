#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
import math
from smach.user_data import UserData


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'





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

ultraNow=0

class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['ultraSound','ultraNow'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
          
            rospy.loginfo("i'm in dummy init var")
            userdata.ultraSound=[]
            userdata.ultraNow=0
            return 'succeeded'
class pinta_ultra_sound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=[],input_keys=['ultraSound'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
            rospy.loginfo(OKGREEN+"i'm in dummy pinta var")
            rospy.loginfo(str(userdata.ultraSound[0]))
            rospy.loginfo(str(userdata.ultraSound[1]))
            rospy.loginfo(str(userdata.ultraSound[2]))
            rospy.loginfo(str(userdata.ultraSound[3])+ENDC)
            return 'succeeded'
        
        
        
     
class process_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['pose_1','pose_2'],
                             output_keys=['pose_status'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
        aux=userdata.pose_1
        aux1=userdata.pose_2
        #aux1=PoseWithCovarianceStamped()
        min_difrerence_x = 0.05
        min_difrerence_y = 0.05
        userdata.pose_status=False
        
        if (aux.pose.position.x+min_difrerence_x)>aux1.pose.position.x and (aux.pose.position.x-min_difrerence_x)<aux1.pose.position.x :
            if (aux.pose.position.y+min_difrerence_y)>aux1.pose.position.y and (aux.pose.position.y-min_difrerence_y)<aux1.pose.position.y :
                userdata.pose_status=True
            else :
                userdata.pose_status=False
        else :
            userdata.pose_status=False
        rospy.loginfo(OKGREEN)
        rospy.loginfo("//////////////////////////////////POSE/////////////////////////////")
        
        rospy.loginfo(str(aux.pose.position.x))
        rospy.loginfo(str(aux.pose.position.y))
        rospy.loginfo(str(aux1.pose.position.x))
        rospy.loginfo(str(aux1.pose.position.y))
        rospy.loginfo(ENDC)
        return 'succeeded'



   
class process_laser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['Laser_msg'],
                             output_keys=['Laser_status'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
        min_distance = 1
        length=len(userdata.Laser_msg.ranges)  
        
        if userdata.Laser_msg.ranges[1]<min_distance and userdata.Laser_msg.ranges[length-1]<min_distance :
            userdata.Laser_status=True
        else :
            userdata.Laser_status=False
        rospy.loginfo(OKGREEN)
        rospy.loginfo("//////////////////////////////////LASER/////////////////////////////")
        
        rospy.loginfo(str(userdata.Laser_msg.ranges[0]))
        rospy.loginfo(str(userdata.Laser_msg.ranges[length-1]))
        rospy.loginfo(ENDC)
        return 'succeeded'


class process_info(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['elevator','no_elevator'],
                             input_keys=['Laser_status','pose_status','ultraSound_status'])
    def execute(self,userdata):
        rospy.loginfo(OKGREEN)
        rospy.loginfo("Pose say     :"+str(userdata.pose_status))
        rospy.loginfo("Ultra sound say:    "+str(userdata.ultraSound_status))
        rospy.loginfo("Laser say:    "+str(userdata.Laser_status))
        rospy.loginfo(ENDC)
        return 'no_elevator'
     
class get_ultra_sound(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succeeded'],input_keys=[],
                             output_keys=['ultraSound_status'])
    def execute(self,userdata):
        Finish =False
        distance=0.5
        aux=Range()
        ultra=[]
        pep = False
        ultra.append(pep)
        ultra.append(pep)
        ultra.append(pep)
        ultra.append(pep)
        ultra.append(pep)
        ultraSound=[]
        
        while Finish==False:
            aux = rospy.wait_for_message('/sonar_base', Range, 60)
            pep=Range()
            pep.range
            
            
            if ultra[0]==False :       
                if (aux.header.frame_id=='base_sonar_02_link'):
                    ultraSound.append(aux)
                    rospy.loginfo("0")
                    ultra[0]=True
            if ultra[1]==False :       
                if (aux.header.frame_id=='base_sonar_05_link'):
                    ultraSound.append(aux)
                    rospy.loginfo("1")
                    ultra[1]=True
            if ultra[2]==False :       
                if (aux.header.frame_id=='base_sonar_08_link'):
                    ultraSound.append(aux)
                    rospy.loginfo("2")
                    ultra[2]=True
            if ultra[3]==False :       
                if (aux.header.frame_id=='base_sonar_11_link'):
                    ultraSound.append(aux)
                    rospy.loginfo("3")
                    ultra[3]=True
            if ultra[0]==True and ultra[1]==True and ultra[2]==True and ultra[3]==True :
                Finish = True
        
        if ultraSound[0].range<distance and ultraSound[1].range<distance and ultraSound[2].range<distance and ultraSound[3].range<distance :
            userdata.ultraSound_status=True
        else :
            userdata.ultraSound_status=False
            rospy.loginfo(OKGREEN)
            rospy.loginfo("////////////////////////////ULTRA/////////////////////////////////////////")
            rospy.loginfo(str(ultraSound[0]))
            rospy.loginfo(str(ultraSound[1]))
            rospy.loginfo(str(ultraSound[2]))
            rospy.loginfo(str(ultraSound[3]))
            rospy.loginfo(ENDC)
        return 'succeeded'
'''     
class get_ultra_sound3(smach.State):
     
    def __init__(self):
        smach.State.__init__(self,outcomes=['succeeded'],input_keys=['ultraSound'],
                             output_keys=['ultraSound_status'])
    def execute(self,userdata):
        userdata.ultraSound_status=True
        return 'succeeded'
        distance=0.5
        Finish =False
        aux=Range()
        ultraNow=0
         
        while Finish==False:
            aux = rospy.wait_for_message('/sonar_base', Range, 60)
            
            
            if ultraNow==0 :       
                if (aux.header.frame_id=='base_sonar_02_link'):
                    ultraSound=[] # i reset the vector
                    ultraSound.append(aux)
                    rospy.loginfo(str(ultraSound[0]))
                    ultraNow=1
            if ultraNow==1 :       
                if (aux.header.frame_id=='base_sonar_05_link'):
                    ultraSound.append(aux)
                    rospy.loginfo(str(ultraSound[1]))
                    ultraNow=2
            if ultraNow==2 :       
                if (aux.header.frame_id=='base_sonar_08_link'):
                    ultraSound.append(aux)
                    rospy.loginfo(str(ultraSound[2]))
                    ultraNow=3
            if ultraNow==3 :       
                if (aux.header.frame_id=='base_sonar_11_link'):
                    ultraSound.append(aux)
                    rospy.loginfo(str(ultraSound[3]))
                    ultraNow=0
                    Finish=True 
        if ultraSound[0]<distance and ultraSound[1]<distance and ultraSound[2]<distance and ultraSound[3]<distance :
            userdata.ultraSound_status=True
        else :
            userdata.ultraSound_status=False
        return 'succeeded'

'''       
                    
                                  
    
class CheckElevator(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'])
        with self:


            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "GET_CURRENT_POSE_1"})
            
            '''
            smach.StateMachine.add('READ_UltraSound_TOPIC',
                                   topic_reader(topic_name='/sonar_base',
                                                topic_type=Range,topic_time_out=60),
                                   transitions={'succeeded':'PROCESS_ULTRA_SOUND'},
                                   remapping={'topic_output_msg': 'ultraSound_msg'})
            '''
            
            smach.StateMachine.add('GET_CURRENT_POSE_1',
                                   get_current_robot_pose(),
                      transitions={'succeeded': 'GET_ULTRA_SOUND'},
                      remapping={'current_robot_pose':'pose_1'})
            
            smach.StateMachine.add('GET_ULTRA_SOUND',
                       get_ultra_sound(),
                      transitions={'succeeded':'READ_Laser_TOPIC'})
            
            smach.StateMachine.add('READ_Laser_TOPIC',
                                   topic_reader(topic_name='/scan_filtered',
                                                topic_type=LaserScan,topic_time_out=60),
                                   transitions={'succeeded':'PROCESS_LASER'},
                                   remapping={'topic_output_msg': 'Laser_msg'})
                        
            smach.StateMachine.add('PROCESS_LASER',
                       process_laser(),
                      transitions={'succeeded': 'GET_CURRENT_POSE_2'})
            
            smach.StateMachine.add('GET_CURRENT_POSE_2',
                                   get_current_robot_pose(),
                      transitions={'succeeded': 'PROCESS_POSE'},
                      remapping={'current_robot_pose':'pose_2'})
            
            smach.StateMachine.add('PROCESS_POSE',
                                   process_pose(),
                      transitions={'succeeded':'PROCESS_INFO'})
                       
            smach.StateMachine.add('PROCESS_INFO',
                                   process_info(),
                      transitions={'elevator':'succeeded','no_elevator':'INIT_VAR'})
            

                             