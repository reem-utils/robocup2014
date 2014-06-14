#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat June 14 16:00:00 2014

@author: Chang Long Zhu
@email: changlongzj@gmail.com
"""


import rospy
import smach
import smach_ros
from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.enter_room import EnterRoomSM
from speech_states.say import text_to_say
from manipulation_states.play_motion_sm import play_motion_sm
from emergency_situation.Get_Person_Desired_Object import Get_Person_Desired_Object
from emergency_situation.Save_People_Emergency import Save_People_Emergency
from emergency_situation.Search_People_Emergency import Search_People_Emergency
from emergency_situation.search_ambulance import Search_Ambulance_Face, Search_Face_Determined
from geometry_msgs.msg import PoseStamped, Pose
from face_states.detect_faces import detect_face

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        print "Dummy state just to change to other state"

        rospy.sleep(1)
        return 'succeeded'

# Class that prepare the value need for nav_to_poi
class prepare_poi_person_emergency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['person_location_coord'], 
            output_keys=['nav_to_coord_goal']) 
    def execute(self,userdata):
        rospy.loginfo('PersonLOcationCooordddddd :::::::: ' + str(userdata.person_location_coord))
        userdata.nav_to_coord_goal = userdata.person_location_coord 

        return 'succeeded'

class Ambulance_Detect_And_Go(smach.StateMachine):
    """
    Executes a SM that does a part of Emergency Situation.
    - Go to the apartment's entry
    - Guide the helper to the person's position
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input_keys:
    @key: emergency_room: name of the room where the emergency is located

    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        sm = smach.StateMachine.__init__(self, 
                                         outcomes=['succeeded', 'preempted', 'aborted'],
                                         input_keys=['person_location_coord'])

        with self:           
            self.userdata.name=""
            smach.StateMachine.add(
                                   'SetHome',
                                   play_motion_sm('home'),
                                   transitions={'succeeded':'Go_to_Entry_Door', 'aborted':'SetHome', 'preempted':'SetHome'})
            
            smach.StateMachine.add(
                'Say_return_Person',
                text_to_say('I am going to the Entry Door for the Ambulance'),
                transitions={'succeeded':'Go_to_Entry_Door', 'aborted':'Go_to_Entry_Door', 'aborted':'Go_to_Entry_Door'})
            
            smach.StateMachine.add(
                'Go_to_Entry_Door',
                nav_to_poi('entry_door_init'),
                transitions={'succeeded':'Wait_for_Ambulance_Person', 'aborted':'Go_to_Entry_Door', 'preempted':'Go_to_Entry_Door'})

            #What is Wait for Ambulance or People Mean? Person detection?
            smach.StateMachine.add(
                'Wait_for_Ambulance_Person',
                #Search_Ambulance_Face(),
                Search_Face_Determined('Where are you ambulance?'),
                transitions={'succeeded':'Say_Ambulance', 'aborted':'Detect_Fail_Init', 'preempted':'Go_to_Entry_Door'})
            smach.StateMachine.add(
                                   'Detect_Fail_Init',
                                   play_motion_sm('home'),
                                   transitions={'succeeded':'Detect_Fail_Execute', 'aborted':'Detect_Fail_Execute', 'preempted':'Detect_Fail_Execute'})
            smach.StateMachine.add(
                 'Detect_Fail_Execute',
                 detect_face(),
                 transitions={'succeeded': 'Say_Ambulance', 'aborted': 'Say_Ambulance'})
            smach.StateMachine.add(
                'Say_Ambulance',
                text_to_say("Thank you for arriving as fast as possible. Please Follow Me, I will guide you to the emergency"),
                transitions={'succeeded':'Prepare_Emergency_Final', 'aborted':'Prepare_Emergency_Final', 'preempted':'Prepare_Emergency_Final'})

            #If Aborted (not supposed to), retry?
            smach.StateMachine.add(
               'Prepare_Emergency_Final',
               prepare_poi_person_emergency(),
               transitions={'succeeded':'Go_to_emergency_room_2', 'aborted':'Go_to_emergency_room_2', 'preempted':'Go_to_emergency_room_2'})

            smach.StateMachine.add(
                'Go_to_emergency_room_2',
                nav_to_coord('/map'),
                transitions={'succeeded':'Wait_state_emergency', 'aborted':'Go_to_emergency_room_2', 'preempted':'Go_to_emergency_room_2'})

            smach.StateMachine.add(
                'Wait_state_emergency',
                DummyStateMachine(),
                transitions={'succeeded':'Say_Finish', 'aborted':'Say_Finish', 'preempted':'Say_Finish'})

            smach.StateMachine.add(
                'Say_Finish',
                text_to_say('Here is the person that is in an emergency situation, please proceed. I helped to save a life.'),
                transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
            

def main():
    rospy.loginfo('Detect_Ambulance')
    rospy.init_node('Detect_Ambulance_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:      
        smach.StateMachine.add(
            'Detect_Ambulance',
            Ambulance_Detect_And_Go(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()