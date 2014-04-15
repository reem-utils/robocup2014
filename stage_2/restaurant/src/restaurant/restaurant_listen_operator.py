#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach





from speech_states.say import text_to_say
from speech_states.say_yes_or_no import SayYesOrNoSM
from speech_states.listen_and_check_word import ListenWordSM
from navigation_states.get_current_robot_pose import get_current_robot_pose 
from speech_states.listen_to import ListenToSM    
import roslib

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



SAY_OK= "OK, i have added this place, we can go for other one"
SAY_REPEAD="Sorry, i dind't understand you, can you repead again"

'''
In this state machine, we will be listening the operator,
and creating new pois of the places that are needed for
proces the orther
'''





class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"I'M inicialitzating the vars of  the resteurant"+ENDC)
        rospy.sleep(2)
        userdata.standard_error="Dummy"
        return 'succeeded'
    


class save_point(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"I Have a new point"+ENDC)
        rospy.sleep(2)
        return 'succeeded'
    
    
class did_you_say(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"DID you say...."+ENDC)
        rospy.sleep(2)
        return 'succeeded'

class proces_Tags(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['new_position','finish',
                                    'aborted','preempted'],input_keys=['asr_userSaid','asr_userSaid_tags'],output_keys=[])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"i'm lokking what tags are"+ENDC)
        rospy.loginfo(OKGREEN+str(userdata.asr_userSaid)+ENDC)
        rospy.loginfo(OKGREEN+str(userdata.asr_userSaid_tags)+ENDC)
        rospy.sleep(2)
        return 'new_position'
    
class ListenOperator(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.grammar_name="restaurant.gram"
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'LISTEN_TO',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            
            
            smach.StateMachine.add('LISTEN_TO',
                                   ListenToSM(),
                                   transitions={'succeeded': 'DID_YOU_SAY',
                                                'aborted': 'LISTEN_TO','preempted':'preempted'})
            
            
        
            smach.StateMachine.add('DID_YOU_SAY',
                                   did_you_say(),
                                   transitions={'succeeded': 'PROCES_TAGS',
                                                'aborted': 'CAN_YOU_REPEAD','preempted':'preempted'})
            
            smach.StateMachine.add('CAN_YOU_REPEAD',
                       text_to_say(SAY_REPEAD),
                       transitions={'succeeded': 'LISTEN_TO',
                                    'aborted': 'LISTEN_TO','preempted':'preempted'})
                        
            smach.StateMachine.add('PROCES_TAGS',
                       proces_Tags(),
                       transitions={'new_position': 'GET_POSE','finish':'succeeded',
                                    'aborted': 'LISTEN_TO','preempted':'preempted'})

            
            smach.StateMachine.add('GET_POSE',
                                   get_current_robot_pose(),
                                   transitions={'succeeded': 'SAVE_POINT',
                                                'aborted': 'GET_POSE','preempted':'preempted'})
            
            #maybe the yaw of the robot we will have to change and say at your right you have....
            smach.StateMachine.add('SAVE_POINT',
                                   save_point(),
                                   transitions={'succeeded': 'SAY_OK',
                                                'aborted': 'aborted','preempted':'preempted'})

            smach.StateMachine.add('SAY_OK',
                                   text_to_say(SAY_OK),
                                   transitions={'succeeded': 'LISTEN_TO',
                                                'aborted': 'LISTEN_TO','preempted':'preempted'})
            
            
            
        
            
            
            
            
            
            
