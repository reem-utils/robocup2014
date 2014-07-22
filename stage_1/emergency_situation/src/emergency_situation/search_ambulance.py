#! /usr/bin/env python
'''
Created on 14/06/2014

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com

'''
import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from manipulation_states.move_head import move_head
from util_states.timeout import TimeOut
from face_states.recognize_face import recognize_face_concurrent
from speech_states.say import text_to_say
from manipulation_states.move_head_form import move_head_form
import random
import tf
import numpy
import copy
import tf.transformations as TT
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point, Quaternion
from util_states.topic_reader import topic_reader
from pal_vision_msgs.msg import Gesture
from face_states.detect_faces import detect_face
from util_states.math_utils import *
from gesture_states.wave_detection_sm import WaveDetection

GESTURE_TOPIC = '/head_mount_xtion/gestures'
final_frame_id = 'base_link'
PUB_1_TOPIC = '/given_pose'
PUB_2_TOPIC = '/transformed_pose'

NUMBER_OF_HEAD_POSES = 3
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['num_iterations'],
                                output_keys=['nav_to_poi_name', 'num_iterations', 'standard_error'])

    def execute(self, userdata):    
        
        if userdata.num_iterations > 2 :
            return 'aborted'
        
        if userdata.num_iterations % 3 == 0:
            userdata.nav_to_poi_name = "point_room_one"
            rospy.loginfo(OKGREEN + "Point_room_one" + ENDC)
        elif userdata.num_iterations % 3 == 1: 
            userdata.nav_to_poi_name = "point_room_two"
            rospy.loginfo(OKGREEN + "Point_room_two" + ENDC)
        else:
            userdata.nav_to_poi_name = "point_room_three"
            rospy.loginfo(OKGREEN + "Point_room_three" + ENDC)
        
        userdata.num_iterations += 1
        
        return 'succeeded'
 
 
class Wait_search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn('PREEMPT REQUESTED -- Returning Preempted in Wait_search State')
            return 'preempted'
        
        rospy.sleep(2)
        rospy.logwarn('PREEMPT NOT REQUESTED -- Returning Preempted in Wait_search State')
        return 'succeeded'

class random_speech_prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                    outcomes=['succeeded'],
                                    input_keys=[],
                                    output_keys=['tts_text'])
    def execute(self, userdata):
        rand_tts = random.randrange(2)
        if rand_tts == 0:
            userdata.tts_text = 'Where are you Ambulance?'
        elif rand_tts == 1:
            userdata.tts_text = "I am trying to locate the Ambulance, it is an Emergency"
        elif rand_tts == 2:
            userdata.tts_text = "Please help me to save the person"
        else:
            userdata.tts_text = "I am looking for you"
        
        return 'succeeded'
        
class random_speech_say(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys=['tts_text'],
                                    output_keys=[])
        with self:
            smach.StateMachine.add('random_prepare',
                                   random_speech_prepare(),
                                   transitions={'succeeded':'say_random'})
            smach.StateMachine.add('say_random',
                                   text_to_say(),
                                   transitions={'succeeded':'succeeded','aborted':'aborted'})

class say_searching_faces(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])
        
        with self:
            self.userdata.loop_iterations = 0
            self.userdata.wave_position = None
            self.userdata.wave_yaw_degree = None
            self.userdata.standard_error = ''
            
            # Concurrence
            sm_conc2 = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                        default_outcome='succeeded',
                                        input_keys=['tts_text', 'head_left_right', 'head_up_down'],
                                        output_keys=[])
            
            with sm_conc2:
                # Search for face
                smach.Concurrence.add('move_head_conc', move_head_form(None, None))
                
                smach.Concurrence.add('say_conc', random_speech_say())
            
            
            smach.StateMachine.add('Concurrence', sm_conc2, 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'aborted', 
                                             'preempted':'aborted'})
            smach.StateMachine.add('Wait_state',
                                   Wait_search(),
                                   transitions={'succeeded':'Prepare_head'})
            smach.StateMachine.add('Prepare_head',
                                   prepare_move_head('normal'),
                                   transitions={'succeeded':'Concurrence'})
            
# gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if BAR finished
    if outcome_map['find_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Find_faces ends" + ENDC)
        return True
 
    if outcome_map['say_search_faces'] == 'succeeded':
        rospy.loginfo(OKGREEN + "Say and Move Head ends" + ENDC)
        return True

    # in all other case, just keep running, don't terminate anything
    return False

def out_cb(outcome_map):
    if outcome_map['find_faces'] == 'succeeded':
        rospy.logwarn('Out_CB = Find Faces Succeeded')

        return 'succeeded'     
    else:
        rospy.logwarn('Out_CB = Else!')
        return 'aborted'



    

class Search_Ambulance_Face(smach.StateMachine):
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
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['name', 'nav_to_poi_name', 'face','face_frame'],
                                     output_keys=['face', 'standard_error', 'face_frame'])

        with self:
            
            self.userdata.num_iterations = 0
            self.userdata.face = None
            self.userdata.wait_time = 5
            
            smach.StateMachine.add(
                                   'Say_Searching',
                                   text_to_say('Right Now I am looking for the Ambulance.'),
                                   transitions={'succeeded': 'Concurrence', 'aborted': 'aborted', 
                                    'preempted': 'preempted'})
            
            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                        default_outcome='succeeded',
                                        input_keys=['name', 'nav_to_poi_name', 'face', 'wait_time'],
                                        output_keys=['face', 'standard_error', 'face_frame'],
                                        child_termination_cb = child_term_cb,
                                        outcome_cb=out_cb)
            
            with sm_conc:
                # Search for face
                smach.Concurrence.add('find_faces', recognize_face_concurrent())
                
                smach.Concurrence.add('say_search_faces', say_searching_faces())

            
            smach.StateMachine.add('Concurrence', sm_conc, 
                                transitions={'succeeded':'succeeded', 
                                             'aborted':'aborted', 
                                             'preempted':'aborted'})


######################################################################################################################

class prepare_move_head(smach.State):
    def __init__(self, head_position):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['loop_iterations', 'head_left_right', 'head_up_down'],
                                output_keys=['head_left_right', 'head_up_down', 'standard_error', 'loop_iterations'])
        self.head_position = head_position

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_HEAD_POSES:
            userdata.loop_iterations = 0
            return 'aborted'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            
            userdata.head_left_right = 1 - userdata.loop_iterations*0.5
            
            if userdata.loop_iterations == 0:
                userdata.head_left_right = 'mid_left'
            elif userdata.loop_iterations == 1:
                userdata.head_left_right = 'center'
            elif userdata.loop_iterations == 2:
                userdata.head_left_right = 'mid_right'

            userdata.head_up_down = self.head_position if self.head_position else 'normal'

            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'

class Search_Face_Determined(smach.StateMachine):
    """
        Search_Wave_SM: This SM searches for any face around the room, 
                        as the robot will move its head from left to right.
                        
        Input Keys:
            None
        Output Keys:

        Required Parameters: 
            None
            
        Outcomes:

    """
    def __init__(self, head_position=None, text_for_wave_searching="I am looking for you."):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=[],
                                 output_keys=['wave_position', 'wave_yaw_degree','standard_error'])
        with self:
            self.userdata.loop_iterations = 0
            self.userdata.wave_position = None
            self.userdata.wave_yaw_degree = None
            self.userdata.standard_error = ''
            
            smach.StateMachine('Dummy',
                               DummyStateMachine(),
                               transitions={'succeeded':'Say_follow'})
            smach.StateMachine('Say_follow',
                               text_to_say('Follow Me Please, I will guide you to the person'),
                               transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            
            smach.StateMachine.add(
                                   'Move_head_prepare',
                                   prepare_move_head(head_position),
                                   transitions={'succeeded': 'move_head', 'aborted': 'aborted', 
                                                'preempted': 'preempted'})
            smach.StateMachine.add(
                                   'move_head',
                                   move_head_form(head_up_down=head_position),
                                   transitions={'succeeded': 'Say_Searching',
                                                'preempted':'Say_Searching',
                                                'aborted':'aborted'})
            smach.StateMachine.add(
                'Say_Searching',
                text_to_say("text_for_wave_searching"),
                transitions={'succeeded':'face_detection', 'aborted':'face_detection', 'preempted':'face_detection'})
           
            #Hard-coded maximum time in order to detect wave 
            smach.StateMachine.add(
                'face_detection',
                detect_face(),
                transitions={'succeeded': 'Say_Found', 'aborted': 'aborted', #TODO before it was Move_head_prepare
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                'Say_Found',
                text_to_say("Oh! I have found you finally."),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
            


                  
        
