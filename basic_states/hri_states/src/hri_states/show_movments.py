#! /usr/bin/env python

import rospy
import smach
from _ast import Return
from docutils.transforms.misc import Transitions


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

from speech_states.say import text_to_say
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.srv import NavigationGoBack, NavigationGoBackRequest, NavigationGoBackResponse
from smach_ros import ServiceState
from navigation_states.nav_to_coord_concurrent import nav_to_coord_concurrent
from pipol_tracker_pkg.msg import personArray,person
from hri_states.search_wave_sm import Search_Wave_SM
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point,PointStamped
from util_states.math_utils import *
from util_states.topic_reader import topic_reader
from manipulation_states.move_head_form import move_head_form
from gesture_states.wave_detection_sm import WaveDetection
from manipulation_states.play_motion_sm import play_motion_sm


Say="Hello, my name is reem, i'm having a realy good time here in Brazil"

SLEEP=1

class waitstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo("waiting")
        rospy.sleep(SLEEP)
        return 'succeeded'
    

class show_movments(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],
                                    output_keys=[],
                                    input_keys=[])
        
        with self:

            
            #maybe i will have to learn again
            smach.StateMachine.add('INIT_VAR',
                                   waitstate(),
                                   transitions={'succeeded': 'play_motion_state',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('home', skip_planning=False),
                transitions={'succeeded': 'WAIT_0',
                             'preempted':'WAIT_0', 
                             'aborted':'WAIT_0'}) 
            
            smach.StateMachine.add('WAIT_0',
                       waitstate(),
                       transitions={'succeeded': 'say_0',
                                    'aborted': 'say_0','preempted':'preempted'}) 
            
            smach.StateMachine.add(
                'say_0',
                text_to_say("hello my name is reem, i'm really happy to be here and show you my movements. I come from Barcelona. I was created by pal robotics. I came here with my team reem at la salle.",wait=False),
                transitions={'succeeded': 'play_motion_state_0', 'aborted': 'play_motion_state_0', 'preempted': 'play_motion_state_0'})

            smach.StateMachine.add(
                'play_motion_state_0',
                play_motion_sm('wave', skip_planning=False),
                transitions={'succeeded': 'WAIT_1',
                             'preempted':'WAIT_1', 
                             'aborted':'WAIT_1'})  
            
            smach.StateMachine.add('WAIT_1',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_1',
                                    'aborted': 'play_motion_state_1','preempted':'preempted'})         
            
            
            smach.StateMachine.add(
                'play_motion_state_1',
                play_motion_sm('arms_t', skip_planning=False),
                transitions={'succeeded': 'WAIT_2',
                             'preempted':'WAIT_2', 
                             'aborted':'WAIT_2'})  

            smach.StateMachine.add('WAIT_2',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_2',
                                    'aborted': 'play_motion_state_2','preempted':'preempted'})             
            
            smach.StateMachine.add(
                'play_motion_state_2',
                play_motion_sm('wave', skip_planning=False),
                transitions={'succeeded': 'WAIT_3',
                             'preempted':'WAIT_3', 
                             'aborted':'WAIT_3'})  
            smach.StateMachine.add('WAIT_3',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_3',
                                    'aborted': 'play_motion_state_3','preempted':'preempted'}) 


            smach.StateMachine.add(
                'play_motion_state_3',
                play_motion_sm('brochure_tray', skip_planning=False),
                transitions={'succeeded': 'WAIT_4',
                             'preempted':'WAIT_4', 
                             'aborted':'WAIT_4'})  
            
            smach.StateMachine.add('WAIT_4',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_4',
                                    'aborted': 'play_motion_state_4','preempted':'preempted'}) 
            
            
            smach.StateMachine.add(
                'play_motion_state_4',
                play_motion_sm('carry_basquet', skip_planning=False),
                transitions={'succeeded': 'WAIT_5',
                             'preempted':'WAIT_5', 
                             'aborted':'WAIT_5'})  
            
            smach.StateMachine.add('WAIT_5',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_5',
                                    'aborted': 'play_motion_state_5','preempted':'preempted'}) 
            
            
            smach.StateMachine.add(
                'play_motion_state_5',
                play_motion_sm('shake_right', skip_planning=False),
                transitions={'succeeded': 'WAIT_6',
                             'preempted':'WAIT_6', 
                             'aborted':'WAIT_6'})  
            
            smach.StateMachine.add('WAIT_6',
                       waitstate(),
                       transitions={'succeeded': 'play_motion_state_6',
                                    'aborted': 'play_motion_state_6','preempted':'preempted'}) 
            
            
            smach.StateMachine.add(
                'play_motion_state_6',
                play_motion_sm('open_arms', skip_planning=False),
                transitions={'succeeded': 'WAIT_0',
                             'preempted':'WAIT_0', 
                             'aborted':'WAIT_0'})  
            




def main():
    rospy.loginfo('show Node')
    rospy.init_node('show')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'wave',
            show_movments(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()



            
            
            
            
            
            
            
            
