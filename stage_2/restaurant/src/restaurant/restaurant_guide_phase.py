#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach

from follow_me.follow_operator import FollowOperator
from speech_states.say import text_to_say
from speech_states.say_yes_or_no import SayYesOrNoSM
from speech_states.listen_and_check_word import ListenWordSM
from restaurant_init_phase import restaurantInit
from restaurant_listen_operator import ListenOperator
from pal_navigation_msgs.msg import NavigationStatus
from pal_navigation_msgs.srv import Acknowledgment,AcknowledgmentRequest,AcknowledgmentResponse
from smach_ros import ServiceState

"""
RESTAURANT_guide.PY

"""


SAY_FINISH_FOLLOWING= "OK, now we can stard ordering 33"
SAY_LETS_GO=" i'M READY, WHEN YOU WANT WE CAN START"

import roslib


DISTANCE_TO_FOLLOW = 1.0
LEARN_PERSON_FLAG = True


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

MAPPING_MODE = 'MAP'
LOCALIZATION_MODE = 'LOC'
DISABLED = 'STOP'
gramar= 'robocup/restaurantGuide'

class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],
            input_keys=[],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"I'M in the restaurant"+ENDC)
        userdata.standard_error="Dummy"
        return 'succeeded'
     


    

class learn_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded',
                                    'aborted','preempted'],output_keys=['in_learn_person'])
    def execute(self,userdata):
        rospy.loginfo("im learning a person")
        userdata.in_learn_person="temporal"
        return 'succeeded'
             
        
class ListenOperator_dummy(smach.State):
    # gets called when ANY child state terminates
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
    def execute(self,userdata):
        rospy.sleep()
        rospy.loginfo("im dummy listen the operator")
        return 'succeeded'
        
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['FOLLOW_ME'] == 'lost':
        
        return True
    
    if outcome_map['LISTEN_OPERATOR_RESTAURANT'] == 'succeeded':
        
        return True
    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['FOLLOW_ME'] == 'lost':
        rospy.loginfo(OKGREEN + "the door its open" + ENDC)
        return 'lost'    
    elif outcome_map['LISTEN_OPERATOR_RESTAURANT'] == 'succeeded':
        rospy.loginfo(OKGREEN + "the operator me go out" + ENDC)
        return 'succeeded'    

    return 'aborted'


#I will be hear waiting for the instruction to go out
class restaurantGuide(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted', 'aborted'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.grammar_name=gramar
            self.userdata.in_learn_person=1 # todo is only for try
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'LEARN_INIT',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            def navigation_MAP_cb(userdata,request):
                update = AcknowledgmentRequest()
                update.input = MAPPING_MODE
                return update
            
            def navigation_LOC_cb(userdata,request):
                update = AcknowledgmentRequest()
                update.input = LOCALIZATION_MODE
                return update
            
            smach.StateMachine.add( 'CHANGE_STATE_MAP',
                                    ServiceState('pal_navigation_sm',Acknowledgment,
                                                 request_cb = navigation_MAP_cb ),
                                    transitions = {'succeeded': 'LEARN_INIT',
                                                    'aborted':'CHANGE_STATE_MAP'})
            smach.StateMachine.add('LEARN_INIT',
                                   restaurantInit(),
                                   transitions={'succeeded': 'CONCURRENCE',
                                                'aborted': 'LEARN_INIT','preempted':'preempted'})
            
        
            
            sm=smach.Concurrence(outcomes=['succeeded', 'lost'],
                                   default_outcome='succeeded',input_keys=["in_learn_person",
                                                                           'grammar_name'],
                                   child_termination_cb = child_term_cb, outcome_cb=out_cb)
                
             
            with sm:
                # it follow the person for long time
                sm.add('FOLLOW_ME',
                                FollowOperator())
                # here it have to listen and put pois in the map
                sm.add('LISTEN_OPERATOR_RESTAURANT',
                                ListenOperator())
                
            smach.StateMachine.add('CONCURRENCE', sm, transitions={'succeeded': 'CHANGE_STATE_STOP',
                                                                   'lost':'CONCURRENCE'})
            
            
            
            smach.StateMachine.add('CHANGE_STATE_STOP',
                                    ServiceState('pal_navigation_sm',Acknowledgment,
                                                 request_cb = navigation_LOC_cb ),
                                    transitions = {'succeeded': 'FINISH',
                                                   'aborted':'CHANGE_STATE_STOP'})
            
            # it say finsih that then we can stard serving orders
            smach.StateMachine.add('FINISH',
                                   text_to_say(SAY_FINISH_FOLLOWING),
                                   transitions={'succeeded': 'succeeded',
                                    'aborted': 'aborted','preempted':'preempted'})
            
    

            
            
            
            
            
            
            
