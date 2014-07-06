#! /usr/bin/env python
'''
Created on 08/03/2014

@author: Roger Boldu

'''
import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from manipulation_states.move_head import move_head
from navigation_states.srv import NavigationTurn, NavigationTurnRequest, NavigationTurnResponse
from util_states.timeout import TimeOut
from face_states.recognize_face import recognize_face_concurrent
from speech_states.say import text_to_say
from manipulation_states.move_head_form import move_head_form
from navigation_states.turn_infinit import turn_infinit


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(1)
        return 'succeeded'

class prepare_poi(smach.State):
    def __init__(self,poi_name):
        smach.State.__init__(self, outcomes=['poi','no_poi'], 
                                input_keys=['nav_to_poi_name'],
                                output_keys=['nav_to_poi_name'])
        self.poi_name=poi_name
    def execute(self, userdata):    
        
        if userdata.nav_to_poi_name == None and self.poi_name==None :
            return 'no_poi'
        else :
            if userdata.nav_to_poi_name == None :
                userdata.nav_to_poi_name=self.poi_name
                return 'poi'
 
 
class Wait_search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn('PREEMPT REQUESTED -- Returning Preempted in Wait_search State')
            return 'preempted'
        
        rospy.sleep(1.5)
        rospy.logwarn('PREEMPT NOT REQUESTED -- Returning Preempted in Wait_search State')
        return 'succeeded'

class say_searching_faces(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

        with self:

            smach.StateMachine.add('Wait_search_1', 
                                   Wait_search(), 
                                   transitions={'succeeded':'Say_search', 'preempted':'preempted'})
            
            smach.StateMachine.add('Say_search', 
                                   text_to_say('I am looking for the referee',wait=False), 
                                   transitions={'succeeded':'Move_head_Left', 'aborted':'aborted'})
            
            smach.StateMachine.add('Move_head_Left',
                                   move_head_form( head_left_right='mid_left', head_up_down='normal'),
                                   transitions={'succeeded':'Wait_search_2', 'aborted':'aborted'})
            
            smach.StateMachine.add('Wait_search_2', 
                                   Wait_search(), 
                                   transitions={'succeeded':'Move_head_Right', 'preempted':'preempted'})
            
            smach.StateMachine.add('Move_head_Right',
                                   move_head_form( head_left_right='mid_right', head_up_down='normal'),
                                   transitions={'succeeded':'Wait_search_3', 'aborted':'aborted'})
            
            smach.StateMachine.add('Wait_search_3', 
                                   Wait_search(), 
                                   transitions={'succeeded':'Move_head_Middle', 'preempted':'preempted'})
            
            smach.StateMachine.add('Move_head_Middle',
                                   move_head_form( head_left_right='center', head_up_down='normal'),
                                   transitions={'succeeded':'succeeded', 'aborted':'aborted'})
 
            
# gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['turn'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Walk_to_poi ends" + ENDC)
        
        return False

    # terminate all running states if BAR finished
    if outcome_map['find_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Find_faces ends" + ENDC)
        return True
    
    if outcome_map['TimeOut'] == 'succeeded':
        rospy.loginfo(OKGREEN + "TimeOut ends" + ENDC)
        return True
    
    if outcome_map['say_search_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Say and Move Head ends" + ENDC)
        return False

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['find_faces'] == 'succeeded':
        rospy.logwarn('Out_CB = Find Faces Succeeded')
        return 'succeeded'    
    elif outcome_map['TimeOut'] == 'succeeded':
        rospy.logwarn('Out_CB = TimeOut finished succeeded')
        return 'endTime'    
    else:
        rospy.logwarn('Out_CB = Else!')
        return 'aborted'



    

class go_find_person(smach.StateMachine):
    """
    The robot goes around a room looking for faces. When it detects the face from
    TC, it stops and return success. 
    
    To do the search we define 3 points
    The robot will go around the 3 points looking for the TC
    When it found it, the state machine return success. Unless it return aborted.

    We need a concurrence state machine:
        -> if face detects the face -> it return success
        -> if walk return success and face aborted -> it return aborted and we need define the new goal
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    """
    def __init__(self,poi_name=None,time_out=90):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['name', 'nav_to_poi_name', 'face','face_frame'],
                                     output_keys=['face', 'standard_error', 'face_frame'])
        self.poi_name=poi_name
        self.time_out=time_out
        with self:

            self.userdata.num_iterations = 0
            self.userdata.face = None
            self.userdata.wait_time = 5
            self.userdata.nav_to_poi_name=None
            self.userdata.face_frame=None
            
            # We define the different points
            smach.StateMachine.add(
                'prepare_poi',
                prepare_poi(self.poi_name),
                transitions={'poi': 'go_poi', 'no_poi': 'Concurrence'}) 
            smach.StateMachine.add(
                                   'Say_Searching',
                                   text_to_say('Right Now I am looking for you.',wait=False),
                                   transitions={'succeeded': 'go_poi', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})
            
            smach.StateMachine.add(
                                   'go_poi',
                                   nav_to_poi(),
                                   transitions={'succeeded': 'Concurrence', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})

            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted', 'endTime'],
                                        default_outcome='succeeded',
                                        input_keys=['name', 'nav_to_poi_name', 'face', 'wait_time'],
                                        output_keys=['face', 'standard_error', 'face_frame'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            
            with sm_conc:
                # Go around the room 
               # smach.Concurrence.add('walk_to_poi', nav_to_poi())                  
                smach.Concurrence.add('turn', turn_infinit('left'))
                # Move head
                smach.Concurrence.add('TimeOut', TimeOut(self.time_out))
                 
                # Search for face
                smach.Concurrence.add('find_faces', recognize_face_concurrent())
                
                smach.Concurrence.add('say_search_faces', say_searching_faces())

            
            smach.StateMachine.add('Concurrence', sm_conc, 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'Say_Finish', 
                                             'endTime': 'Say_Finish',
                                             'preempted':'Say_Finish'})

                  
            smach.StateMachine.add(
                                   'Say_Finish',
                                   text_to_say('I have finish with no exit the search'),
                                   transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})
            
            
            
