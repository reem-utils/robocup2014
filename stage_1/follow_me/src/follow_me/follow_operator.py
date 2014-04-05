#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach

#from track_operator import TrackOperator
from smach_ros import ServiceState, SimpleActionState
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from util_states.math_utils import *
from tf.transformations import quaternion_from_euler
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_coord_concurrent import nav_to_coord_concurrent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util_states.topic_reader import topic_reader
from follow_me.msg import tracker_people # TODO: maybe we have to change it

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



MOVE_BASE_TOPIC_GOAL = "/move_base/goal"



class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['time_last_found'])
    def execute(self, userdata):
            userdata.time_last_found=rospy.Time.now()
            rospy.sleep(1)
            rospy.loginfo("i'm in dummy init var")
            return 'succeeded'
# its the tracker learn person...
        
        # its the tracker learn person...
class filter_and_process(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['find_it','not_found', 'no_pausible'],
                             input_keys=['tracking_msg','tracking_msg_filtered'],
                             output_keys=['tracking_msg_filtered'])
    def execute(self, userdata):
            
            # if the message is perfect i d'ont have to do anything, 
            #only return if it's in the message or not
            aux=tracker_people()
            aux.pose.position.x
            aux.pose.position.y
            #userdata.tracking_msg_filtered=tracker_people()
            userdata.tracking_msg_filtered=userdata.tracking_msg.pose    
            
            #userdata.tracking_msg_filtered='hello'
            rospy.loginfo("i'm in the dummy filter and process state")
            return 'find_it'
# it resset the time from last found, thats because i have find it 
class reset_occluded_timer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['time_last_found'])
    def execute(self, userdata):
         
            userdata.time_last_found=rospy.Time.now()
            rospy.loginfo("i'm in dummy reset_occluded_timer")
            return 'succeeded'
        
        #it will have to look how many time it pass from the last found_it
class no_follow(smach.State):
    def __init__(self,time_occluded=30):
        smach.State.__init__(self, outcomes=['lost','occluded'],input_keys=['time_last_found'])
    def execute(self, userdata):
            aux=rospy.Time.now()
      
            rospy.loginfo("i'm in dummy learning face")
            if (aux.secs-userdata.time_last_found)>self.time_occluded :
                return 'lost'
            else :
                return 'occluded'
        
       
class calculete_goal(smach.State):
    def __init__(self, distanceToHuman=0.9):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=['tracking_msg_filtered','nav_to_coord_goal'],
                             output_keys=['nav_to_coord_goal'])
        self.distanceToHuman=distanceToHuman
    def execute(self, userdata):

        #Calculating vectors for the position indicated
        new_pose = Pose()
        new_pose.position.x = userdata.tracking_msg_filtered.position.x
        new_pose.position.y = userdata.tracking_msg_filtered.position.y
        unit_vector = normalize_vector(new_pose.position)
        position_distance = vector_magnitude(new_pose.position)
        rospy.loginfo(" Position data from Reem to person:")
        rospy.loginfo(" Position vector : " + str(new_pose.position))
        rospy.loginfo(" Unit position vector : " + str(unit_vector))
        rospy.loginfo(" Position vector distance : " + str(position_distance))

        """
If person is closer than the distance given, we wont move but we might rotate.
We want that if the person comes closer, the robot stays in the place.
Thats why we make desired distance zero if person too close.
"""


        distance_des = 0.0
        if position_distance >= self.distanceToHuman: 
            distance_des = position_distance - self.distanceToHuman
        else:
            rospy.loginfo(" Person too close => not moving, just rotate")
        #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

        rospy.loginfo(' Final robot movement data:')
        rospy.loginfo(' Distance from robot center to person : ' + str(position_distance))
        rospy.loginfo(' Person and Reem wanted distance (distance to human) : ' + str(self.distanceToHuman))
        rospy.loginfo(' Distance that REEM will move towards the person : ' + str(distance_des))
        rospy.loginfo(' Degrees that REEM will rotate : ' + str(alfa_degree))

       # nav_goal_msg.header.stamp = rospy.Time.now()
        #nav_goal_msg.header.frame_id = "/base_link"
        #nav_goal_msg.pose.position = dist_vector
        #nav_goal_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, alfa))
        #rospy.loginfo(' This is the Nav Goal We send to REEM: ' + str(nav_goal_msg))
        #userdata.nav_to_coord_goal= nav_goal_msg
 
        userdata.nav_to_coord_goal = [new_pose.position.x, new_pose.position.y, alfa]
        return 'succeeded'

        
        
        
        
        
        
        
        
        # it will send the cord
class send_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],input_keys=['nav_goal_msg'],
                             output_keys=['nav_goal_msg'])
        self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, MoveBaseGoal)
    def execute(self, userdata):
            
            rospy.loginfo("i'm in dummy send_goal state")
            self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, MoveBaseAction)
            self.pub.publish(userdata.nav_goal_msg)
            return 'succeeded'

class debug(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['nav_goal_msg','tracking_msg_filtered','tracking_msg'])
    def execute(self, userdata):
            rospy.loginfo("i'm in dummy debug state")
            return 'succeeded'




class FollowOperator(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, distToHuman=0.9,time_occluded=30):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'lost'],
            input_keys=["in_learn_person"])

        

        with self:
            
#TODO i don't know if it's the currect form to stop de face tracking
            #smach.StateMachine.add('DISABLE_FACE_TRACKING',
             #                          ServiceState('/personServer/faceTracking/stop'),
              #                         transitions={'succeeded': 'FIX_HEAD_POSITION'})
            #self. userdata.tracking_msg=Pose()
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "READ_TRACKER_TOPIC"})

            smach.StateMachine.add('READ_TRACKER_TOPIC',
                                   topic_reader(topic_name='/people_tracker/person',
                                                topic_type=tracker_people,topic_time_out=60),
                                   transitions={'succeeded':'FILTER_AND_PROCESS',
                                                'aborted':'READ_TRACKER_TOPIC',
                                                'preempted':'READ_TRACKER_TOPIC'},
                                   remapping={'topic_output_msg': 'tracking_msg'})

            smach.StateMachine.add('FILTER_AND_PROCESS',
                                   filter_and_process(),
                                   transitions={'find_it': 'I_KNOW',
                                                'not_found': 'I_DONT_KNOW', 
                                                'no_pausible':'READ_TRACKER_TOPIC'})
            smach.StateMachine.add('I_KNOW',
                       reset_occluded_timer(),
                       transitions={'succeeded': 'CALCULETE_GOAL'})
            
            # hear we will comprobate the time, if its greater than LOST_TIME it will return Lost
            smach.StateMachine.add('I_DONT_KNOW',
                       no_follow(time_occluded),
                       transitions={'lost': 'lost',
                                    'occluded': 'READ_TRACKER_TOPIC'})
           # /move_base_simple/goal
            smach.StateMachine.add('CALCULETE_GOAL',
                       calculete_goal(distToHuman),
                       transitions={'succeeded': 'SEND_GOAL',
                                    'aborted': 'READ_TRACKER_TOPIC'})
            
            smach.StateMachine.add('SEND_GOAL',
                       nav_to_coord_concurrent('/base_link'),
                       transitions={'succeeded':'DEBUG', 'preempted':'DEBUG', 'aborted':'DEBUG'})    
            # it have to desaper 
            smach.StateMachine.add('DEBUG',
                       debug(),
                       transitions={'succeeded': 'READ_TRACKER_TOPIC'})                       