#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
import math


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

ultraNow=0

class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['ultraSound','ultraNow'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
            rospy.sleep(1)
            rospy.loginfo("i'm in dummy init var")
            userdata.ultraSound=[]
            userdata.ultraNow=0
            return 'succeeded'
class pinta(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=[],input_keys=['ultraSound'])
    def execute(self, userdata):
            #userdata.ultra_sound_filtered=22
            rospy.sleep(1)
            rospy.loginfo(OKGREEN+"i'm in dummy pinta var")
            rospy.loginfo(str(userdata.ultraSound[0]))
            rospy.loginfo(str(userdata.ultraSound[1]))
            rospy.loginfo(str(userdata.ultraSound[2]))
            rospy.loginfo(str(userdata.ultraSound[3])+ENDC)
            rospy.sleep(5)
            return 'succeeded'
class process_ultraSound(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['no_finish','finish'],input_keys=['ultraSound_msg','ultraSound','ultraNow'],
                             output_keys=['ultraSound','ultraNow'])
       
    def execute(self,userdata):
        '''
        ultrasound tipe
        uint8 ULTRASOUND=0
        uint8 INFRARED=1
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        uint8 radiation_type
        float32 field_of_view
        float32 min_range
        float32 max_range
        float32 range
        '''
        
        aux=userdata.ultraSound_msg
        
        if userdata.ultraNow==0 :       
                if (aux.header.frame_id=='base_sonar_02_link'):
                    userdata.ultraSound=[] # i reset the vector
                    userdata.ultraSound.append(aux)
                    rospy.loginfo(str(userdata.ultraSound_msg))
                    userdata.ultraNow=1
                    return 'no_finish'
        if userdata.ultraNow==1 :       
                if (aux.header.frame_id=='base_sonar_05_link'):
                    userdata.ultraSound.append(aux)
                    rospy.loginfo(str(userdata.ultraSound_msg))
                    userdata.ultraNow=2
                    return 'no_finish'
        if userdata.ultraNow==2 :       
                if (aux.header.frame_id=='base_sonar_08_link'):
                    userdata.ultraSound.append(aux)
                    rospy.loginfo(str(userdata.ultraSound_msg))
                    userdata.ultraNow=3
                    return 'no_finish'
        if userdata.ultraNow==3 :       
                if (aux.header.frame_id=='base_sonar_11_link'):
                    userdata.ultraSound.append(aux)
                    rospy.loginfo(str(userdata.ultraSound_msg))
                    userdata.ultraNow=0
                    return 'finish'    
        
        #userdata.ultraNow=0        
        return 'no_finish'                                    
                
    
class CheckElevator(smach.StateMachine):
    def __init__(self, distToHuman=0.9):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],
                                    input_keys=['in_personTrackingData'])
        with self:


            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "READ_TRACKER_TOPIC"})

            smach.StateMachine.add('READ_TRACKER_TOPIC',
                                   topic_reader(topic_name='/sonar_base',
                                                topic_type=Range,topic_time_out=60),
                                   transitions={'succeeded':'PROCESS_ULTRA_SOUND'},
                                   remapping={'topic_output_msg': 'ultraSound_msg'})
            smach.StateMachine.add('PROCESS_ULTRA_SOUND',
                       process_ultraSound(),
                      transitions={'no_finish': 'READ_TRACKER_TOPIC','finish':'print'})
            
            smach.StateMachine.add('print',
                       pinta(),
                      transitions={'succeeded': 'succeeded'})
                             