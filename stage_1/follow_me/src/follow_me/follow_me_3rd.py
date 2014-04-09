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

SAY_COME_NEAR="CAN YOU APROACH A LITTLE BIT"
SAY_LETS_GO="OK LETS GO AGAIN"
SAY_GO_AGAIN="OK LETS GO AGAIN"

class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        rospy.sleep(2)
        rospy.loginfo(OKGREEN+"i'm in the 3rd part of the robocup"+ENDC)
        return 'succeeded'

# i have to create a navigation state that go backford
class go_back(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute (self,userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in the dummy state of go back")
        return 'succeeded'

#hear it will have to look if i can find the person
class where_is_it(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['i_dont_know','ok_lets_go','preempted'], input_keys=['in_learn_person'])
    def execute(self,userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in a dumy state where i'm  looking if i know the person")
        return 'ok_lets_go'
    
    
    
class learn_again(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                              output_keys=['in_learn_person'])
    def execute(self,userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in the learning dummy state, it's learn again")
        return 'succeeded'

# hear i will send a goal whit my orientation and realy far
class create_nav_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
    def execute(self,userdata):
        rospy.loginfo("i'm creating a a realy far goal, becouse i lost the operator")
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
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
    def execute(self,userdata):
        rospy.loginfo("I'm creating a goal with the gesture position")
        return'succeeded'
    
    
    
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
                                    output_keys=['standard_error'],input_keys=['in_learn_person'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standar_error="ok"
            
            #maybe i will have to learn again
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'GO_BACK',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            # it will go out from the lift
            smach.StateMachine.add('GO_BACK',
                       go_back(),
                       transitions={'succeeded': 'WHERE_IS_IT',
                                    'aborted': 'aborted','preempted':'preempted'})
            
                        # it will go out from the lift
            smach.StateMachine.add('WHERE_IS_IT',
                       where_is_it(),
                       transitions={'i_dont_know':'SAY_COME_NEAR','ok_lets_go':'SAY_LETS_GO','preempted':'preempted'})
            
            #this is when i don't remember the person
            smach.StateMachine.add('SAY_COME_NEAR',
                                   text_to_say(SAY_COME_NEAR),
                                   transitions={'succeeded':'LEARN_AGAIN','aborted':'LEARN_AGAIN','preempted':'preempted'})
            
            #hear i will learn the peron another time
            smach.StateMachine.add('LEARN_AGAIN',
                                   learn_again(),
                                   transitions={'succeeded':'SAY_LETS_GO',
                                                'aborted':'SAY_COME_NEAR','preempted':'preempted'})
            
            
            
            smach.StateMachine.add('SAY_LETS_GO',
                                   text_to_say(SAY_LETS_GO),
                                   transitions={'succeeded':'TRACK_OPERATOR','aborted':'aborted','preempted':'preempted'})  
            
            
            smach.StateMachine.add('TRACK_OPERATOR',FollowOperator(),transitions={'succeeded':'succeeded',
                                               'lost':'LOST_CREATE_GOAL','preempted':'preempted'})
            
            
            
            # i will prepere the navigation goal becouse i have lost the person   
            #the goal have to be a realy far goal   
            smach.StateMachine.add('LOST_CREATE_GOAL',create_nav_goal(),
                                   transitions={'succeeded':'SEARCH_OPERATOR',
                                               'aborted':'SEARCH_OPERATOR','preempted':'preempted'})
            
            
            
            sm=smach.Concurrence(outcomes=['gesture_recongize', 'nav_finisheed','preempted'],
                        default_outcome='nav_finisheed',input_keys=["in_learn_person"],
                        child_termination_cb = child_term_cb,
                        outcome_cb=out_cb_follow,output_keys=[])
            
            with sm:
                sm.userdata.standar_error="ok"
                #this goal will be a realy far goal
                smach.Concurrence.add('SEND_GOAL',
                                nav_to_coord())
 
                smach.Concurrence.add('SEARCH_OPERATOR_GESTURE',
                                gesture_recogniton())
            
            smach.StateMachine.add('SEARCH_OPERATOR', sm,
                                     transitions={'gesture_recongize':'CREATE_GESTURE_NAV_GOAL',
                                                 'nav_finisheed':'SEARCH_OPERATOR','preempted':'preempted'})
           
        
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
                                   recognize_person(),
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
    elif outcome_map['SEARCH_OPERATOR'] == 'succeeded':
        return 'nav_finisheed'             
            
            
            
            
            
            
            
            
            
            
