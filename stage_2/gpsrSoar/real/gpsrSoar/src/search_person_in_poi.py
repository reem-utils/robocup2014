#! /usr/bin/env python
'''
Created on 08/03/2014

@author: Sergi Xavier Ubach Pallas
@email: sxubach@gmail.com

'''
import rospy
import smach
from smach_ros import ServiceState
#from navigation_states.turn import turn
from manipulation_states.move_head import move_head
from util_states.timeout import TimeOut
from face_states.recognize_face import recognize_face_concurrent
from speech_states.say import text_to_say
from manipulation_states.move_head_form import move_head_form
from navigation_states.srv import *
from httplib2 import Response



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
    
class NamePreparation(smach.State):
    def __init__(self,person_name):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['name'], 
            output_keys=['name'])

    def execute(self,person_name, userdata):
        if person_name = 'person':
            userdata.name = ''
        else:
            userdata.name = person_name
            
        return 'succeeded'

 
class Wait_search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):        
        rospy.sleep(1.5)
        rospy.logwarn('PREEMPT NOT REQUESTED -- Returning succeeded in Wait_search State')
        return 'succeeded'
    
class turn(smach.StateMachine):
    def __init__(self, angle = 120):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted','aborted'],
            input_keys=[])
        
        self.angle= angle
        self.iterations = 0

        with self:
            
            self.userdata.angle = self.angle
            
            def iterator(userdata, response):
                self.iterations = self.iterations + 1
                rospy.logwarn("ITERATIONS = " + str(self.iterations))
                if self.iterations > 2:
                    return 'preempted'
                return 'succeeded'
                    
            def nav_turn_start(userdata, request):
                turn_request = NavigationTurnRequest()
                turn_request.degree = self.angle
                turn_request.enable = True
                return turn_request
            
            smach.StateMachine.add('turn',
                                   ServiceState('/turn',
                                                NavigationTurn,
                                                request_cb = nav_turn_start,
                                                response_cb = iterator),
                                   transitions={'succeeded':'succeeded','aborted' : 'turn','preempted':'preempted'})      
            

class say_searching_faces(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

        with self:
            
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_text = ''
            self.userdata.tts_lang = 'en_US'

            smach.StateMachine.add('Wait_search_1', 
                                   Wait_search(), 
                                   transitions={'succeeded':'Say_search', 'preempted':'preempted'})
            
            smach.StateMachine.add('Say_search', 
                                   text_to_say('I am looking for the person'), 
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

    # terminate all running states if turn finished with outcome succeeded
    if outcome_map['turn'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Turn ends" + ENDC)
#         if outcome_map['say_search_faces'] != 'succeeded':        
#             return False
    if outcome_map['turn'] == 'preempted':
        rospy.loginfo(OKGREEN + "Turn ends" + ENDC)
        return True

    # terminate all running states if BAR finished
    if outcome_map['find_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Find_faces ends" + ENDC)
        return True
    
#     if outcome_map['TimeOut'] == 'succeeded':
#         rospy.loginfo(OKGREEN + "TimeOut ends" + ENDC)
#         return True
    
    if outcome_map['say_search_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Say and Move Head ends" + ENDC)
        return True

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['find_faces'] == 'succeeded' :
        rospy.logwarn('Out_CB = Find Faces Succeeded')
        return 'succeeded'    
    
    if outcome_map['turn'] == 'preempted':
        rospy.logwarn('Out_CB = Find Faces Failed')
        return 'aborted'    
#     elif outcome_map['TimeOut'] == 'succeeded':
#         rospy.logwarn('Out_CB = TimeOut finished succeeded')
#         return 'endTime'  
    elif outcome_map['say_search_faces'] == 'succeeded':
        return 'preempted'
    else:
        rospy.logwarn('Out_CB = Else!')
        return 'aborted'




class SearchPersonSM(smach.StateMachine):
    """
    The robot looks around a room looking for faces. When it detects the face from
    the "name" person, it stops and return success. 
    
    To do the search we turn 3 times looking for the person
    When it found it, the state machine return success. Unless it return aborted.

    We need a concurrence state machine:
        -> if face detects the face -> it return success
        -> if turn return success and face aborted -> it return aborted and we need define the new goal
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    """
    def __init__(self,person_name):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['face','face_frame'],
                                     output_keys=['face', 'standard_error', 'face_frame'])
        
        with self:
            self.userdata.name = person_name
            self.userdata.num_iterations = 0
            self.userdata.face = None
            self.userdata.wait_time = 50
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_text = ''
            self.userdata.tts_lang = 'en_US'
                        

            smach.StateMachine.add(
                                   'Name_Preparation',
                                   NamePreparation(person_name),
                                   transitions={'succeeded': 'Say_Searching', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})            

            smach.StateMachine.add(
                                   'Say_Searching',
                                   text_to_say('Right Now I am looking for you.'),
                                   transitions={'succeeded': 'Concurrence', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})
            
            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted', 'endTime'],
                                        default_outcome='succeeded',
                                        input_keys=['name', 'face', 'wait_time'],
                                        output_keys=['face', 'standard_error', 'face_frame'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            
            with sm_conc:
                # Go around the room 
                smach.Concurrence.add('turn', turn(120))
          
                # Move head
                #smach.Concurrence.add('TimeOut', TimeOut(1000))
                 
                # Search for face
                smach.Concurrence.add('find_faces', recognize_face_concurrent())
                
                smach.Concurrence.add('say_search_faces', say_searching_faces())

            
            smach.StateMachine.add('Concurrence', sm_conc, 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'aborted', 
                                             'endTime': 'aborted',
                                             'preempted':'Say_Changing_Poi'})

                  
            smach.StateMachine.add('Say_Changing_Poi',
                                   text_to_say('I am going to change my position so I can search for faces'),
                                   transitions={'succeeded': 'Say_Searching', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})
            
            