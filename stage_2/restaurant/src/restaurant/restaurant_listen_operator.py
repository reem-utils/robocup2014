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
from geometry_msgs.msg import PoseWithCovarianceStamped
from roslaunch.loader import rosparam

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



SAY_OK= "OK, i have added this place, we can go for other one"
SAY_REPEAT="Sorry, i dind't understand you, can you repeat again"

'''
In this state machine, we will be listening the operator,
and creating new pois of the places that are needed for
process the order
'''





#here i have to delete all params
class init_var(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],
            input_keys=['standard_error'],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"Init vars of  the restaurant"+ENDC)
        userdata.standard_error="Dummy"
        return 'succeeded'
    


class save_point(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],
            input_keys=['current_robot_pose','object','current_robot_yaw'],output_keys=[])

    def execute(self, userdata):
        aux=userdata.current_robot_pose
        aux.pose.position.x
        aux.pose.position.y
        rospy.loginfo(OKGREEN+"I Have a new point"+ENDC)
        
        value=["submap_0",userdata.object,aux.pose.position.x,
               aux.pose.position.y,userdata.current_robot_yaw]
        
        rospy.loginfo(OKGREEN+str(value)+ENDC)
        rospy.set_param("/restaurant/submap_0/"+str(userdata.object),value)
        return 'succeeded'
    
    
class did_you_say(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=[],output_keys=[])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"DID you say...."+ENDC)

        return 'succeeded'

class proces_Tags(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['new_position','finish',
                            'aborted','preempted'],
                             input_keys=['asr_userSaid','asr_userSaid_tags'],
                             output_keys=['object'])

    def execute(self, userdata):
        rospy.loginfo(OKGREEN+"i'm looking what tags are"+ENDC)
        rospy.loginfo(OKGREEN+str(userdata.asr_userSaid)+ENDC)
        rospy.loginfo(OKGREEN+str(userdata.asr_userSaid_tags)+ENDC)
        userdata.object=userdata.asr_userSaid # it means that in this place it have a coke
        
        if userdata.asr_userSaid=="finish" :
            rospy.logwarn("-------------------------------------i'm have a finish order")
            return 'finish'
        else :
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
            self.userdata.asr_userSaid=None
            self.userdata.asr_userSaid_tags=None
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'LISTEN_TO',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            
            smach.StateMachine.add('LISTEN_TO',
                                   ListenToSM(),
                                   transitions={'succeeded': 'PROCES_TAGS',
                                                'aborted': 'LISTEN_TO','preempted':'preempted'})
            
            
        
            smach.StateMachine.add('CAN_YOU_REPEAT',
                       text_to_say(SAY_REPEAT),
                       transitions={'succeeded': 'LISTEN_TO',
                                    'aborted': 'LISTEN_TO','preempted':'preempted'})
                        
            smach.StateMachine.add('PROCES_TAGS',
                       proces_Tags(),
                       transitions={'new_position': 'GET_POSE','finish':'succeeded',
                                    'aborted': 'CAN_YOU_REPEAT','preempted':'preempted'})

            
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
            
            
            
        
            
            
            
            
            
            
