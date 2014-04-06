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
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute (self,userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in the dummy state of go back")
        return 'succeeded'

#hear it will have to look if i can find the person
class where_is_it(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['i_dont_know','ok_lets_go'], input_keys=['in_learn_person'])
    def execute(self,userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in a dumy state where i'm  looking if i know the person")
        return 'ok_lets_go'
    
    
    
class learn_again(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
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
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        
    def execute(self,userdata):
        rospy.loginfo("I'm in the dummy of recognition gesture")
        
        return 'succeeded'
    
    
 # In this state it will create the nav goal   
class create_nav_gesture_goal(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self,userdata):
        rospy.loginfo("I'm creating a goal with the gesture position")
        return'succeeded'
    
    
    
class recognize_person(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
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
            self.userdata.standar_error="ok"
            
            #maybe i will have to learn again
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            # it will go out from the lift
            smach.StateMachine.add('GO_BACK',
                       go_back(),
                       transitions={'succeeded': 'succeeded',
                                    'aborted': 'aborted','preempted':'preempted'})
            
                        # it will go out from the lift
            smach.StateMachine.add('WHERE_IS_IT',
                       where_is_it(),
                       transitions={'succeeded':'SAY_COME_NEAR','ok_lets_go':'SAY_LETS_GO'})
            
            #this is when i don't remember the person
            smach.StateMachine.add('SAY_COME_NEAR',
                                   text_to_say(SAY_COME_NEAR),
                                   transitions={'i_dont_know':'LEARN_AGAIN','aborted':'LEARN_AGAIN'})
            
            #hear i will learn the peron another time
            smach.StateMachine.add('LEARN_AGAIN',
                                   learn_again(),
                                   transitions={'succeeded':'SAY_LETS_GO',
                                                'aborted':'SAY_COME_NEAR'})
            
            
            
            smach.StateMachine.add('SAY_LETS_GO',
                                   text_to_say(SAY_LETS_GO),
                                   transitions={'succeeded':'TRACK_OPERATOR','aborted':'aborted'})  
            
            
            
            
            smach.StateMachine.add('TRACK_OPERATOR',
                                   FollowOperator(),
                                   transtions={'succeeded':'succeeded',
                                               'lost':'LOST','preempted':'preempted'})
            
            
            
            # i will prepere the navigation goal becouse i have lost the person   
            #the goal have to be a realy far goal   
            smach.StateMachine.add('LOST',create_nav_goal(),
                                   transtions={'succeeded':'succeeded',
                                               'lost':'LOST','preempted':'preempted'})
            
            
            
            
            sm=smach.Concurrence(outcomes=['gesture_recongize', 'nav_finisheed'],
                        default_outcome='nav_finisheed',input_keys=["in_learn_person"],
                        child_termination_cb = child_term_cb,
                        outcome_cb=out_cb_follow,output_keys=[])
            
            with sm:
                #this goal will be a realy far goal
                smach.Concurrence.add('SEND_GOAL',
                                nav_to_coord())
 
                smach.Concurrence.add('SEARCH_OPERATOR_GESTURE',
                                gesture_recogniton())
            
            smach.StateMachine.add('SEARCH_OPERATOR', sm,
                                     transitions={'gesture_recongize':'GO_TO_GHESTURE',
                                                 'nav_finisheed':'aborted','preempted':'succeeded'})
           
        
            smach.StateMachine.add('CREATE_GESTURE_NAV_GOAL',
                                   create_nav_gesture_goal(),
                                   transtions={'succeeded':'succeeded',
                                               'aborted':'aborted','preempted':'preempted'})
            
            
            # in this state i will have lost the person, now i have to send a Goal realy far, and 
            smach.StateMachine.add('CREATE_GESTURE_NAV_GOAL',
                                   create_nav_gesture_goal(),
                                   transtions={'succeeded':'GO_TO_GESTURE',
                                               'aborted':'aborted','preempted':'preempted'})
            
            # in this state i will have lost the person, now i have to send a Goal realy far, and 
            smach.StateMachine.add('GO_TO_GESTURE',
                                   nav_to_coord('/base_link'),
                                   transtions={'succeeded':'RECOGNIZE_PERSON',
                                               'aborted':'GO_TO_GESTURE','preempted':'preempted'})  
            
            #when i'm whit the person i have to look if it's the person
            smach.StateMachine.add('RECOGNIZE_PERSON',
                                   recognize_person(),
                                   transtions={'succeeded':'succeeded',
                                               'aborted':'aborted','preempted':'preempted'})               
            
            
            
            smach.StateMachine.add('RECOGNIZE_PERSON',
                                   text_to_say(SAY_GO_AGAIN),
                                   transtions={'succeeded':'succeeded',
                                               'aborted':'aborted','preempted':'preempted'}) 
            
            # hear i finish the state
            smach.StateMachine.add('FOLLOW_AGAIN',
                       FollowOperator(),
                       transtions={'succeeded':'succeeded',
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
            
            
            
            
            
            
            
            
            
            
