#! /usr/bin/env python

import rospy
import smach
from _ast import Return
from docutils.transforms.misc import Transitions


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

from speech_states.say import text_to_say
from follow_operator import FollowOperator
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.srv import NavigationGoBack, NavigationGoBackRequest, NavigationGoBackResponse
from smach_ros import ServiceState
from navigation_states.nav_to_coord_concurrent import nav_to_coord_concurrent
from follow_me.srv import EnableCheckElevatorRequest, EnableCheckElevator, EnableCheckElevatorResponse
from follow_me.follow_learn import LearnPerson
from pipol_tracker_pkg.msg import personArray,person
from hri_states.search_wave_sm import Search_Wave_SM
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from util_states.math_utils import *
from util_states.topic_reader import topic_reader

SAY_COME_NEAR="CAN YOU APROACH A LITTLE BIT"
SAY_LETS_GO="OK LETS GO AGAIN"
SAY_GO_AGAIN="OK LETS GO AGAIN"

METERSBACK=3



class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['nav_to_coord_goal'],
            output_keys=['nav_to_coord_goal'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"i'm in the 3rd part of the robocup"+ENDC)
        userdata.nav_to_coord_goal = [0,0,0] # after go out we send this goal to finish the problem
        return 'succeeded'

# i have to create a navigation state that go backford
class go_back(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute (self,userdata):
        rospy.loginfo("i'm in the dummy state of go back")
        return 'succeeded'

#hear it will have to look if i can find the person
class where_is_it(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['i_dont_know','ok_lets_go','preempted'],
                              input_keys=['in_learn_person','tracking_msg'])
    def execute(self,userdata):
        find=False

        if self.preempt_requested():
            return 'preempted'
    
        for user in userdata.tracking_msg.peopleSet :

            if userdata.in_learn_person.targetId == user.targetId :
            #    print FAIL + "*************** userdata.in_learn_person.targetId == user.targetId HAPPENED" + ENDC
                userdata.tracking_msg_filtered=user
                find=True
                    
        
        if find :
            #rospy.logerr("\n\nid i am looking for is:  "+ str(userdata.in_learn_person))
            # i want that be like 3 or 4
            if  (userdata.tracking_msg_filtered.targetStatus & person.OCCLUDDED):
                return 'ok_lets_go'
            else :
                return 'ok_lets_go'
        else :
            
            return 'i_dont_know'
    


# hear i will send a goal whit my orientation and realy far
class create_nav_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],output_keys=['nav_to_coord_goal'])
    def execute(self,userdata):
        userdata.nav_to_coord_goal=[2,0,0]
        rospy.loginfo("i'm creating a a realy far goal, because i lost the operator")
        return 'succeeded'
    

#this state itt have to be make oot
class gesture_recogniton(smach.State):
    def __init_(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self,userdata):
        rospy.loginfo("I'm in the dummy of recognition gesture")
        
        return 'succeeded'
    
    
 # In this state it will create the nav goal   
class create_nav_gesture_goal(smach.State):
    
    def __init__(self, distanceToHuman=0.4):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['wave_position', 'wave_yaw_degree','nav_to_coord_goal'],
                             output_keys=['nav_to_coord_goal'])

        self.distanceToHuman=distanceToHuman
    def execute(self, userdata):
        #self.distanceToHuman=DISTANCE_HUMAN
        #Calculating vectors for the position indicated
        new_pose = Pose()
        new_pose.position.x = userdata.wave_position.point.x
        new_pose.position.y = userdata.wave_position.point.y
        
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
            #alfa = math.atan2(userdata.tracking_msg_filtered.y,userdata.tracking_msg_filtered.x)
        #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
        
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

 
        userdata.nav_to_coord_goal = [dist_vector.x, dist_vector.y, alfa]
                
        if self.preempt_requested():
            return 'preempted'
        
        return 'succeeded'

    
# gets called when ANY child state terminates
def child_term_cb(outcome_map):



    if outcome_map['SEARCH_OPERATOR_GESTURE'] == 'succeeded':
        rospy.loginfo(OKGREEN + "perfect i have find the gesture!!!" + ENDC)
        return True


    if outcome_map['SEND_GOAL'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Navigation have been finished... i dind't find the operator" + ENDC)
        return True

    return False

def out_cb_follow(outcome_map):

    if outcome_map['SEARCH_OPERATOR_GESTURE'] == 'succeeded':
        return 'gesture_recongize'    
    # it means that i finish the navigation goal and i have not findit
    elif outcome_map['SEND_GOAL'] == 'succeeded':
        return 'nav_finisheed'             
            
    
class recognize_person(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
    def execute(self,userdata):
        rospy.loginfo("i'm recognizing if the person is the correct person")
        return 'succeeded'


#Defining the state Machine of Learn Person
class follow_me_3rd(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error'],
                                    input_keys=['in_learn_person'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standar_error="ok"
            self.userdata.word_to_listen=None
            self.userdata.in_learn_person=person()
            self.userdata.in_learn_person.targetId=1
            print "HEY, THIS IS self.userdata.in_learn_person: " + str(self.userdata.in_learn_person)
            
            def go_back_request(userdata,request):
                start_request = NavigationGoBackRequest()
                start_request.enable=True
                start_request.meters=METERSBACK
                return start_request
            
            #maybe i will have to learn again
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'GO_BACK',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            # i stop the service
            def Cheack_Elevator_Stop(userdata, request):
                start_request = EnableCheckElevatorRequest()
                start_request.enable=False
                return start_request
            
            smach.StateMachine.add('STOP_CHECK_ELEVATOR',
                                  ServiceState('/check_elevator/enable',
                                  EnableCheckElevator,
                                  request_cb = Cheack_Elevator_Stop),
                                  transitions={'succeeded':'succeeded',
                                               'preempted':'succeeded',
                                                'aborted':'succeeded'})
            # it will go from the lift
            smach.StateMachine.add('GO_BACK',
                                    ServiceState('/reverse',
                                    NavigationGoBack,
                                    request_cb = go_back_request),
                                    transitions={'succeeded':'READ_TRACKER_TOPIC','aborted' : 'READ_TRACKER_TOPIC','preempted':'preempted'})
            
# i don't understant why is this goal here, i imagine for stop
            smach.StateMachine.add('SEND_GOAL',
                       nav_to_coord_concurrent('/base_link'),
                       transitions={'succeeded':'READ_TRACKER_TOPIC', 'aborted':'WHERE_IS_IT','preempted':'preempted'})
            
            
            
            smach.StateMachine.add('READ_TRACKER_TOPIC',
                                   topic_reader(topic_name='/pipol_tracker_node/peopleSet',
                                                topic_type=personArray,topic_time_out=60),
                                   transitions={'succeeded':'WHERE_IS_IT',
                                                'aborted':'READ_TRACKER_TOPIC',
                                                'preempted':'preempted'},
                                   remapping={'topic_output_msg': 'tracking_msg'})
             # it will have to look if it can find the person, know it's only one try
            smach.StateMachine.add('WHERE_IS_IT',
                       where_is_it(),
                       transitions={'i_dont_know':'SAY_COME_NEAR','ok_lets_go':'SAY_LETS_GO','preempted':'preempted'})
            
            #this is when i don't remember the person
            smach.StateMachine.add('SAY_COME_NEAR',
                                   text_to_say(SAY_COME_NEAR),
                                   transitions={'succeeded':'LEARN_AGAIN','aborted':'LEARN_AGAIN','preempted':'preempted'})
            
            #hear i will learn the peron another time
            smach.StateMachine.add('LEARN_AGAIN',
                                   LearnPerson(),
                                   transitions={'succeeded':'SAY_LETS_GO',
                                                'aborted':'SAY_COME_NEAR','preempted':'preempted'})
            
            
            
            smach.StateMachine.add('SAY_LETS_GO',
                                   text_to_say(SAY_LETS_GO),
                                   transitions={'succeeded':'TRACK_OPERATOR','aborted':'aborted','preempted':'preempted'})  
            
            
            smach.StateMachine.add('TRACK_OPERATOR',
                                   FollowOperator(learn_if_lost=False),
                                   transitions={'succeeded':'succeeded',
                                               'lost':'LOST_CREATE_GOAL','preempted':'preempted'})
            
            
            
            # i will prepere the navigation goal becouse i have lost the person   
            #the goal have to be a realy far goal   
            smach.StateMachine.add('LOST_CREATE_GOAL',create_nav_goal(),
                                   transitions={'succeeded':'SEARCH_OPERATOR',
                                               'aborted':'SEARCH_OPERATOR','preempted':'preempted'})
            
            
            
            sm=smach.Concurrence(outcomes=['gesture_recongize', 'nav_finisheed','preempted'],
                        default_outcome='nav_finisheed',
                        input_keys=["in_learn_person",'nav_to_coord_goal'],
                        output_keys=['wave_position', 'wave_yaw_degree','standard_error'],
                        child_termination_cb = child_term_cb,
                        outcome_cb=out_cb_follow)
            
            with sm:
                sm.userdata.standar_error="ok"
                #this goal will be a realy far goal
                smach.Concurrence.add('SEND_GOAL',
                                nav_to_coord("/base_link"))
 
                smach.Concurrence.add('SEARCH_OPERATOR_GESTURE',
                                Search_Wave_SM())
            
            smach.StateMachine.add('SEARCH_OPERATOR', sm,
                                     transitions={'gesture_recongize':'CREATE_GESTURE_NAV_GOAL',
                                                 'nav_finisheed':'SEARCH_OPERATOR_GESTURE_2','preempted':'preempted'})
           
        
            smach.StateMachine.add('SEARCH_OPERATOR_GESTURE_2',
                                   Search_Wave_SM(),
                                   transitions={'succeeded':'CREATE_GESTURE_NAV_GOAL',
                                               'aborted':'SEARCH_OPERATOR_GESTURE_2','preempted':'preempted', 'end_searching':'SEARCH_OPERATOR_GESTURE_2'})
            smach.StateMachine.add('CREATE_GESTURE_NAV_GOAL',
                                   create_nav_gesture_goal(),
                                   transitions={'succeeded':'GO_TO_GESTURE',
                                               'aborted':'aborted','preempted':'preempted'})



            #if the navigation goal it's impossible it will be heare allways 
            smach.StateMachine.add('GO_TO_GESTURE',
                                   nav_to_coord('/base_link'),
                                   transitions={'succeeded':'RECOGNIZE_PERSON',
                                               'aborted':'RECOGNIZE_PERSON','preempted':'preempted'})  
            
            #when i'm whit the person i have to look if it's the person
            smach.StateMachine.add('RECOGNIZE_PERSON',
                                   LearnPerson(),
                                   transitions={'succeeded':'SAY_GO_AGAIN',
                                               'aborted':'aborted','preempted':'preempted'})               
            
            
            
            smach.StateMachine.add('SAY_GO_AGAIN',
                                   text_to_say(SAY_GO_AGAIN),
                                   transitions={'succeeded':'FOLLOW_AGAIN',
                                               'aborted':'SEARCH_OPERATOR','preempted':'preempted'}) 
            
            # hear i finish the state
            smach.StateMachine.add('FOLLOW_AGAIN',
                       FollowOperator(),
                       transitions={'succeeded':'succeeded',
                                   'lost':'aborted','preempted':'preempted'}) 
            
            
            
            

            
            
            
            
            
            
            
            
            
