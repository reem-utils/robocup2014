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
from pipol_tracker_pkg.msg import personArray,person
from speech_states.say import text_to_say

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


NEAR=1
OCLUDED=2
LOST=3
OKI=4


FREQ_FIND=0.2 # publish a 2 HZ only if i send a goal
FREQ_NOT_FIND=0.1 #freq if i'm occluded or lost
MOVE_BASE_TOPIC_GOAL = "/move_base/goal"
#DISTANCE_HUMAN=0.2

TIME_SPEACK_OCLUDED=5
TIME_OCLUDED_SAY="o!! it's a long time that you are occluded, don't leave without me"

TIME_SPEACK_OK=15
TIME_OK_SAY="we are doing fine"

TIME_NEAR_SAY="why aren't you moving?"
TIME_SPEACK_NEAR=15

LOST_SENTENCE = "I'm sorry, I lost you"

class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'],
                             output_keys=[])
    def execute(self, userdata):
            #userdata.in_learn_person=1 # now is hartcoded
            
            if self.preempt_requested():
                return 'preempted'
            return 'succeeded'

# Here we have to take the message that we need, chosing the correct ID
class filter_and_process(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['find_it','not_find','occluded','preempted'],
                             input_keys=['tracking_msg','in_learn_person', 'tracking_msg_filtered'],
                             output_keys=['tracking_msg_filtered','in_learn_person'])
    def execute(self, userdata):
        find=False

        if self.preempt_requested():
            return 'preempted'
    
        for user in userdata.tracking_msg.peopleSet :
           # print OKGREEN  + "userdata.in_learn_person.targetId == user.targetId:" + ENDC
           # print "if " + str( userdata.in_learn_person.targetId) + " == " + str( user.targetId)
            if userdata.in_learn_person.targetId == user.targetId :
            #    print FAIL + "*************** userdata.in_learn_person.targetId == user.targetId HAPPENED" + ENDC
                userdata.tracking_msg_filtered=user
                find=True
                    
        
        if find :
            #rospy.logerr("\n\nid i am looking for is:  "+ str(userdata.in_learn_person))
            # i want that be like 3 or 4
            if  (userdata.tracking_msg_filtered.targetStatus & person.OCCLUDDED):
                return 'occluded'
            else :
                return 'find_it'
        else :
            
            return 'not_find'
     
        #it will have to look how many time it pass from the last found_it
class no_follow(smach.State):
    def __init__(self,time_occluded=30):
        smach.State.__init__(self, outcomes=['lost','preempted'],input_keys=['time_last_found'])
    def execute(self, userdata):
           
            self.feadback=LOST
            rospy.loginfo("i'm in dummy Not following, i have lost ") 
            rospy.sleep(FREQ_NOT_FIND)
            if self.preempt_requested():
                return 'preempted'
            return 'lost'

#this is a occluded time... for the moment we dont send a goal if we are in occluded
class occluded_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['occluded','succeeded','preempted'],
                             input_keys=['tracking_msg_filtered','feadback'],output_keys=['feadback'])
    def execute(self, userdata):
            userdata.feadback=OCLUDED
            rospy.loginfo("i'm in dummy occluded")
            rospy.sleep(FREQ_NOT_FIND)
            if self.preempt_requested():
                return 'preempted'
            return 'occluded'

       
       
class calculate_goal(smach.State):
    def __init__(self, distanceToHuman=0.4):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['tracking_msg_filtered','nav_to_coord_goal','feadback'],
                             output_keys=['nav_to_coord_goal','feadback'])

        self.distanceToHuman=distanceToHuman
    def execute(self, userdata):
        #self.distanceToHuman=DISTANCE_HUMAN
        #Calculating vectors for the position indicated
        new_pose = Pose()
        
        new_pose.position.x = userdata.tracking_msg_filtered.x
        new_pose.position.y = userdata.tracking_msg_filtered.y
        
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

        distance_des = 0.0 #TODO: I DON? UNDERSTAND allwas it will be a movment
        
        if position_distance >= self.distanceToHuman: 
            distance_des = position_distance - self.distanceToHuman
            userdata.feadback=OKI
            rospy.logwarn("----neu feadback :  "+str(userdata.feadback))
            #alfa = math.atan2(userdata.tracking_msg_filtered.y,userdata.tracking_msg_filtered.x)
        else:
            rospy.loginfo(OKGREEN+" Person too close => not moving, just rotate"+ENDC)
            userdata.feadback=NEAR
            rospy.logwarn("----neu feedback NEAR:  "+str(userdata.feadback))
        #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
        
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

        rospy.loginfo(' Final robot movement data:')
        rospy.loginfo(' Distance from robot center to person : ' + str(position_distance))
        rospy.loginfo(' Person and Reem wanted distance (distance to human) : ' + str(self.distanceToHuman))
        rospy.loginfo(' Distance that REEM will move towards the person : ' + str(distance_des))
        rospy.loginfo(' Degrees that REEM will rotate : ' + str(alfa_degree))

 
        userdata.nav_to_coord_goal = [dist_vector.x, dist_vector.y, alfa]
                
        if self.preempt_requested():
            return 'preempted'
        return 'succeeded'

        
        
        # it will send the cord
class freq_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'],
                             input_keys=['nav_goal_msg'],
                             output_keys=['nav_goal_msg'])
    
    def execute(self, userdata):
            
            if self.preempt_requested():
                return 'preempted'
            rospy.loginfo("i'm in dummy send_goal state")
            rospy.sleep(FREQ_FIND)
            return 'succeeded'
        

#TODO: i have to print all the "boxes" look the last year document
class debug(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'],
                             input_keys=['nav_goal_msg',
                                         'tracking_msg_filtered','tracking_msg'])
    def execute(self, userdata):
            rospy.loginfo("i'm in dummy debug state")
            return 'succeeded'

class feadback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['liftime','no_liftime'],
                             input_keys=['tracking_msg_filtered','feadback','old_status']
                             ,output_keys=['tts_text','feadback','old_status'])
        
    def execute(self, userdata):
        userdata.old_status=userdata.old_status
        userdata.feadback=userdata.feadback
        
        rospy.logwarn("-------olf status : "+str(userdata.old_status)+ "mew satusss :  "+str(userdata.feadback))
        msg=person()
        # it means that is not the first time
        if userdata.old_status == userdata.feadback :
            
            if (userdata.old_status==OCLUDED) and (rospy.get_rostime().secs-self.time_voice_init.secs)>TIME_SPEACK_OCLUDED :       
                self.time_voice_init=rospy.get_rostime()
                userdata.tts_text=TIME_OCLUDED_SAY
                return 'liftime'
            if (userdata.old_status==OKI) and (rospy.get_rostime().secs-self.time_voice_init.secs)>TIME_SPEACK_OK :       
                self.time_voice_init=rospy.get_rostime()
                userdata.tts_text=TIME_OK_SAY
                return 'liftime'
            if (userdata.old_status==NEAR) and (rospy.get_rostime().secs-self.time_voice_init.secs)>TIME_SPEACK_NEAR :       
                self.time_voice_init=rospy.get_rostime()
                userdata.tts_text=TIME_NEAR_SAY
                return 'liftime'
        else :
            self.time_voice_init=rospy.get_rostime()
            userdata.old_status=userdata.feadback
            userdata.old_status = userdata.feadback
            return 'no_liftime'

        return 'no_liftime'


class FollowOperator(smach.StateMachine):
    
    '''
    This is a follow operator state.
    This state needs a in_learn_poerson that is a 
    id that will have to look, i don't know the skill perfectly
    
    @input= userdata.in_learn_person
    @Optional input = distToHuman
    @Optional input = time_occluded
    the optional parameters have to be pas in ()
    @Output= I't not have a output
    
    @Outcomes= 'succeeded' : never will be succed because it's infinit
                'lost' : if the time of occluded it's more than time_occluded
                        for this reason is that we have lost
                'preempted': It will be util to do a concurrent state machine
    
    '''
    #Its an infinite loop track_Operator

    def __init__(self, distToHuman=0.4,time_occluded=30):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'lost','preempted'],
            input_keys=['in_learn_person'])

        

        with self:
            self.userdata.old_status=0
            self.userdata.feadback=0 # that means that we don't have feadback
            self.userdata.standard_error='OK'
            #self.userdata.in_learn_person=1
            self.userdata.word_to_listen=None
            
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang='en_US'

            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "READ_TRACKER_TOPIC",
                                               'preempted':'preempted'})

            smach.StateMachine.add('READ_TRACKER_TOPIC',
                                   topic_reader(topic_name='/pipol_tracker_node/peopleSet',
                                                topic_type=personArray,topic_time_out=60),
                                   transitions={'succeeded':'FILTER_AND_PROCESS',
                                                'aborted':'READ_TRACKER_TOPIC',
                                                'preempted':'preempted'},
                                   remapping={'topic_output_msg': 'tracking_msg'})
            
            # in this state i filter the message and give only a person_msg
            # I_KNOW is that i have find the targed_id in personArray
            # not_found is that i don't
            smach.StateMachine.add('FILTER_AND_PROCESS',
                                   filter_and_process(),
                                   transitions={'find_it': 'CALCULATE_GOAL',
                                                'occluded':'OCCLUDED_PERSON',
                                                'not_find': 'I_DONT_KNOW',
                                                'preempted':'preempted'})
            
            # this state now it's dummy, maybe we will like to do something if it's to long time
            smach.StateMachine.add('OCCLUDED_PERSON',
                       occluded_person(),
                       transitions={'occluded': 'DEBUG',
                                    'succeeded':'CALCULATE_GOAL',
                                    'preempted':'preempted'})
            
            # this state now it's dummy, maybe we will like to do something before throw in the towel
            smach.StateMachine.add('I_DONT_KNOW',
                       text_to_say(LOST_SENTENCE),
                       transitions={'succeeded': 'lost','preempted':'preempted', 'aborted':'lost'})
           
            smach.StateMachine.add('CALCULATE_GOAL',
                       calculate_goal(distToHuman),
                       transitions={'succeeded': 'SEND_GOAL',
                                    'aborted': 'READ_TRACKER_TOPIC','preempted':'preempted'})
            
            smach.StateMachine.add('SEND_GOAL',
                       nav_to_coord_concurrent('/base_link'),
                       transitions={'succeeded':'FREQ_SENDING', 
                                    'aborted':'DEBUG','preempted':'preempted'})
            
            smach.StateMachine.add('FREQ_SENDING',
                       freq_goal(),
                       transitions={'succeeded':'DEBUG','preempted':'preempted'})
            
            smach.StateMachine.add('CREATE_FEADBACK',
                       feadback(),
                       transitions={'liftime':'SAY_FEADBACK','no_liftime':'READ_TRACKER_TOPIC'})
            
            smach.StateMachine.add('SAY_FEADBACK',
                       text_to_say(),
                       transitions={'succeeded':'READ_TRACKER_TOPIC','aborted':'SAY_FEADBACK',
                                    'preempted':'preempted'})
            
            # it have to desaper 
            smach.StateMachine.add('DEBUG',
                       debug(),
                       transitions={'succeeded': 'CREATE_FEADBACK','preempted':'preempted'})                       