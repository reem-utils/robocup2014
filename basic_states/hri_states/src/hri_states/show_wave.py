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

SLEEP=3

class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],
            output_keys=[])

    def execute(self, userdata):
        
        return 'succeeded'
    
class waitstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
    def execute(self, userdata):
        rospy.loginfo("waiting")
        rospy.sleep(SLEEP)
        return 'succeeded'
    
class head_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['wave_position', 'wave_yaw_degree','standard_error'],
                             output_keys=['point_to_look'])
    def execute(self, userdata):
        userdata.point_to_look=PointStamped()
        userdata.point_to_look = PointStamped()
        userdata.point_to_look.header.frame_id = 'base_link'
        userdata.point_to_look.point.x = userdata.wave_position.x
        userdata.point_to_look.point.y = userdata.wave_position.y
        userdata.point_to_look.point.z = userdata.wave_position.z
        return 'succeeded'

#Defining the state Machine of Learn Person
class show_wave(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],
                                    output_keys=[],
                                    input_keys=[])
        
        with self:

            
            
            
            #maybe i will have to learn again
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'look_wave',
                                                'aborted': 'aborted','preempted':'preempted'})

                        #maybe i will have to learn again
            smach.StateMachine.add('look_wave',
                                   WaveDetection(),
                                   transitions={'succeeded': 'say_hello',
                                                'aborted': 'look_wave','preempted':'preempted'})
            
            smach.StateMachine.add(
                 'say_hello',
                 text_to_say(Say,wait=False),
                 transitions={'succeeded': 'wait_state', 'aborted': 'wait_state'}) 
            
            smach.StateMachine.add(
                'play_motion_state',
                play_motion_sm('wave', skip_planning=True),
                transitions={'succeeded': 'wait_state',
                             'preempted':'wait_state', 
                             'aborted':'wait_state'})  

            

            smach.StateMachine.add(
                                'wait_state',
                                waitstate(),
                                transitions={'succeeded': 'look_wave', 'aborted': 'look_wave', 
                                'preempted': 'look_wave'})

            
        
            
            
            
            

def main():
    rospy.loginfo('Wave Detection show Node')
    rospy.init_node('wave_detection_node_show')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'wave',
            show_wave(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()



            
            
            
            
            
            
            
            
