#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach
import math





from speech_states.say import text_to_say
from speech_states.say_yes_or_no import SayYesOrNoSM
from speech_states.listen_and_check_word import ListenWordSM
from navigation_states.get_current_robot_pose_mapping import get_current_robot_pose_mapping 
from speech_states.listen_to import ListenToSM    
import roslib
from geometry_msgs.msg import PoseWithCovarianceStamped
from roslaunch.loader import rosparam
from speech_states.parser_grammar import parserGrammar
from hri_states.acknowledgment import acknowledgment

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

GRAMMAR_NAME="robocup/restaurantGuide"



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
            input_keys=['current_robot_pose','objectName','objectOrientation','current_robot_yaw'],
            output_keys=[])

    def execute(self, userdata):
        aux=userdata.current_robot_pose
        aux.pose.position.x
        aux.pose.position.y
        yaw=userdata.current_robot_yaw
        rospy.loginfo(OKGREEN+"I Have a new point"+ENDC)
        if (userdata.objectOrientation== 'right') :
            yaw=yaw+(math.pi/2)
        if (userdata.objectOrientation== 'left'):
            yaw=yaw-(math.pi/2)
        if (userdata.objectOrientation== 'back'):
            yaw=yaw-math.pi
        if (userdata.objectOrientation=='front'):
            yaw=yaw
        
        
        
        
        value=["submap_0",userdata.objectName,aux.pose.position.x,
               aux.pose.position.y,yaw]
        
        rospy.loginfo(OKGREEN+str(value)+ENDC)
        rospy.set_param("/mmap/poi/submap_0/"+str(userdata.objectName),value)
        #"/restaurant/submap_0/"+str(userdata.objectName)
        return 'succeeded'
    
    

class proces_Tags(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['new_position','finish','aborted', 'preempted'], 
                                input_keys=["asr_userSaid", 'asr_userSaid_tags', 'object'],
                                output_keys=['objectName','objectOrientation','tts_text'])
        #self.tags = parserGrammar(GRAMMAR_NAME)
        
       

    def execute(self, userdata):
        listTags = userdata.asr_userSaid_tags
        rospy.loginfo(OKGREEN+"i'm looking what tags are    "+ str(listTags) +ENDC)
        rospy.loginfo(OKGREEN+str(userdata.asr_userSaid)+ENDC)
        #rospy.loginfo(OKGREEN+"TAGS: "+str(self.tags)+ENDC)
        #rospy.loginfo(OKGREEN+str(userdata.asr_userSaid_tags)+ENDC)
        #userdata.object=userdata.asr_userSaid # it means that in this place it have a coke
        userdata.tts_text= "i have listened that "+userdata.asr_userSaid
        if "guide" in userdata.asr_userSaid :
            rospy.logwarn("-------------------------------------i'm have a finish order")
            return 'finish'
        
        
         
        #Process tags
       # userdata.object_array = []
        locationValue=None
        objectValue=None
        locationValue = [tag for tag in listTags if tag.key == 'direction']
        objectValue = [tag for tag in listTags if tag.key == 'objects']
        
        
        if objectValue and locationValue:
            userdata.objectOrientation = locationValue[0].value
            userdata.objectName = objectValue[0].value
            return 'new_position'  
        else:
            rospy.logerr("Object or Location not set")
            return 'aborted'  
              
        #objectValue =          self.tags[0][1]
        #locationValue = self.tags[1][1]
        #objectsRecognized=None
        #locationRecognized=None
        #phrase = []
        
#         phrase = userdata.asr_userSaid.split()
#         rospy.logwarn(str(objectValue))
#         rospy.logwarn(str(locationValue))
#         rospy.logerr(phrase)
#         locationWord=False
#         objectWord=False
#               
#         for word in phrase:
#             
#             if not locationWord:
#                 for element in locationValue:
#                     if element == word:
#                         locationRecognized=element
#                         locationWord = True
#                         break
#                     
#             if not objectWord:
#                 rospy.logwarn(str(word))
#                 for element in objectValue:
#                     if element == word:
#                         objectsRecognized = element
#                         objectWord = True
#                         break
#                     
#             if objectWord and locationWord :
#                 userdata.objectName=objectsRecognized
#                 userdata.objectOrientation=locationRecognized
#                 return 'new_position'
#         
#         if not objectWord or objectsRecognized :
#             return 'aborted'
#       
#         return 'aborted'
        

class ListenOperator(smach.StateMachine):



    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['grammar_name'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.nav_to_poi_name=None
            self.userdata.standard_error='OK'
            self.userdata.asr_userSaid=None
            self.userdata.asr_userSaid_tags=None
            self.userdata.objectName=""
            self.userdata.objectOrientation=""
            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'feedback_listen',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add('feedback_listen',
                                   text_to_say("i am listening"),
                                   transitions={'succeeded': 'LISTEN_TO',
                                                'aborted': 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add('LISTEN_TO',
                                   ListenToSM(GRAMMAR_NAME),
                                   transitions={'succeeded':'PROCES_TAGS',# 'PROCES_TAGS',
                                                'aborted': 'CAN_YOU_REPEAT','preempted':'preempted'})

            
        
            smach.StateMachine.add('CAN_YOU_REPEAT',
                       acknowledgment(tts_text=SAY_REPEAT,type_movement='no'),
                       transitions={'succeeded': 'LISTEN_TO',
                                    'aborted': 'LISTEN_TO','preempted':'preempted'})
                        
            smach.StateMachine.add('PROCES_TAGS',
                       proces_Tags(),
                       transitions={'new_position': 'feedback_repead','finish':'succeeded',
                                    'aborted': 'CAN_YOU_REPEAT','preempted':'preempted'})
            
            smach.StateMachine.add('feedback_repead',
                       text_to_say(),
                       transitions={'succeeded': 'GET_POSE',
                                    'aborted': 'aborted','preempted':'preempted'})

            
            smach.StateMachine.add('GET_POSE',
                                   get_current_robot_pose_mapping(),
                                   transitions={'succeeded': 'SAVE_POINT',
                                                'aborted': 'GET_POSE',
                                                'preempted':'preempted'})
            
            #maybe the yaw of the robot we will have to change and say at your right you have....
            smach.StateMachine.add('SAVE_POINT',
                                   save_point(),
                                   transitions={'succeeded': 'SAY_OK',
                                                'aborted': 'aborted','preempted':'preempted'})

            smach.StateMachine.add('SAY_OK',
                                   acknowledgment(tts_text=SAY_OK,type_movement='yes'),
                                   transitions={'succeeded': 'LISTEN_TO',
                                                'aborted': 'LISTEN_TO','preempted':'preempted'})
            
            
            
        
            
            
            
            
            
            
