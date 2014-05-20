#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

Created on 16/04/2014
"""

import rospy
import smach

from navigation_states.nav_to_poi import nav_to_poi
#from navigation_states.enter_room import EnterRoomSM
from object_grasping_states.recognize_object import recognize_object
from speech_states.listen_to import ListenToSM
from speech_states.say_yes_or_no import SayYesOrNoSM
from speech_states.say import text_to_say
from speech_states.parser_grammar import parserGrammar
from speech_states.activate_asr import ActivateASR
from speech_states.deactivate_asr import DeactivateASR
from speech_states.read_asr import ReadASR
from hri_states.acknowledgment import acknowledgment

# Constants
NUMBER_OF_ORDERS = 3
GRAMMAR_NAME = 'robocup/deliver'

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])
        self.text = text

    def execute(self, userdata):
        rospy.loginfo(self.text)
        rospy.sleep(3)
        return 'succeeded'

class process_restaurant_order(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['asr_userSaid_tags', 'asr_userSaid', 'object_array'],
                                output_keys=['standard_error', 'object_array', 'tts_text'])
       # self.tag = parserGrammar(GRAMMAR_NAME)

    def obtain_object_class(self, objectName):
                    
        objList = rospy.get_param("mmap/object/information")
        className = None
        
        for key,value in objList.iteritems():
            if value[1] == objectName:
                className = value[2]
                break    
                        
        return className
    
    def execute(self, userdata):
        # We obtain the next element of the array
        # {objectName, objectLocation, deliveryLocation}
        # In this parser we suppose that the object A and B always goes in location 1 
        # and Object C in location 2  
        
        # Recognize all the objects
#         objectValue = self.tag[4][1]
#         locationValue = self.tag[3][1]
#         objectsRecognized = []
#         locationRecognized = ""
#         userdata.object_array = []
#         locationFound = False
#         phrase = userdata.asr_userSaid.split()
#         rospy.logwarn("objects:========:   "+str(objectValue))
#         rospy.logwarn("locationValue:========:   "+str(locationValue))
# 
#         for word in phrase:
#             wordFound = False
#             
#             for element in objectValue:
#                 
#                 if element == word:
#                     rospy.logfatal(str(word))
#                     objectsRecognized.append(element)
#                     wordFound = True
#                     break
#                     
#             if not wordFound:
#                 for element in locationValue:
#                     if element == word:
#                         rospy.logfatal(str(word))
#                         locationRecognized = element
#                         locationFound = True
#                         break
#                     
#             if locationFound:
#                 rospy.logwarn(str(objectsRecognized))
#                 for obj in objectsRecognized:
#                     classObj = self.obtain_object_class(obj)
#                     userdata.object_array.append([obj, classObj, locationRecognized])
#                 objectsRecognized = []
#                 locationFound = False
# 
#        
        listTags = userdata.asr_userSaid_tags
         
        #Process tags
        userdata.object_array = []
        actiontag = [tag for tag in listTags if tag.key == 'action']
        objectAtag = [tag for tag in listTags if tag.key == 'objectA']
        objectBtag = [tag for tag in listTags if tag.key == 'objectB']
        objectCtag  = [tag for tag in listTags if tag.key == 'objectC']
        location1tag = [tag for tag in listTags if tag.key == 'location1']
        location2tag = [tag for tag in listTags if tag.key == 'location2']
     
        for i in range(len(actiontag)):
            if actiontag and actiontag[i].value == 'goto' and objectAtag and objectBtag and objectCtag and location1tag and location2tag:
                objectA = objectAtag[i].value
                objectB = objectBtag[i].value
                objectC = objectCtag[i].value
                location1 = location1tag[i].value
                location2 = location2tag[i].value
                   
                classA = self.obtain_object_class(objectA)
                classB = self.obtain_object_class(objectB)
                classC = self.obtain_object_class(objectC)
                 
                userdata.object_array.append([objectA, classA, location1])
                userdata.object_array.append([objectB, classB, location1])
                userdata.object_array.append([objectC, classC, location2])
       
        if userdata.object_array:
            
            userdata.tts_text = ("I Got it! You asked me to,     firsstly, fetch the "
                                 +str(userdata.object_array[0][0])+ ", that is a "
                                 +str(userdata.object_array[0][1])+ ", and the "
                                 +str(userdata.object_array[1][0])+ ", that is a " 
                                 +str(userdata.object_array[1][1])+ " and i have to put it in the "
                                 +str(userdata.object_array[0][2])+ ". Afterwards, I will fetch the " 
                                 +str(userdata.object_array[2][0])+ " that is a " 
                                 +str(userdata.object_array[2][1])+ ", and take it to the " 
                                 +str(userdata.object_array[2][2])) + ".  It is correct?"
                                          
            rospy.logwarn("I Got it! You asked me to, firstly, fetch the "
                                 +str(userdata.object_array[0][0])+ " and the "
                                 +str(userdata.object_array[1][0])+ " and i have to put it in the "
                                 +str(userdata.object_array[0][2])+ ". Afterwards, I will fetch the " 
                                 +str(userdata.object_array[2][0]) + ", and take it to the " + str(userdata.object_array[2][2]))
            return 'succeeded'
            
        return 'aborted'#         objectValue = self.tag[4][1]
#         locationValue = self.tag[3][1]
#         objectsRecognized = []
#         locationRecognized = ""
#         userdata.object_array = []
#         locationFound = False
#         phrase = userdata.asr_userSaid.split()
#         rospy.logwarn("objects:========:   "+str(objectValue))
#         rospy.logwarn("locationValue:========:   "+str(locationValue))
# 
#         for word in phrase:
#             wordFound = False
#             
#             for element in objectValue:
#                 
#                 if element == word:
#                     rospy.logfatal(str(word))
#                     objectsRecognized.append(element)
#                     wordFound = True
#                     break
#                     
#             if not wordFound:
#                 for element in locationValue:
#                     if element == word:
#                         rospy.logfatal(str(word))
#                         locationRecognized = element
#                         locationFound = True
#                         break
#                     
#             if locationFound:
#                 rospy.logwarn(str(objectsRecognized))
#                 for obj in objectsRecognized:
#                     classObj = self.obtain_object_class(obj)
#                     userdata.object_array.append([obj, classObj, locationRecognized])
#                 objectsRecognized = []
#                 locationFound = False
# 
#        

class RestaurantOrder(smach.StateMachine):
    """
    Executes a SM that does the Restaurant Order.

    The robot ask for the order, confirm and process the information. 
    It returns an array with the information {objectName, objectLocation, deliveryLocation}
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    Output keys:
        object_array: Array with the information of the order
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['object_array'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=None
            self.userdata.grammar_name = None
            self.userdata.type_movment = None
            self.userdata.tts_wait_before_speaking = 0
            self.userdata.tts_lang = None
            self.userdata.standard_error='OK'
            self.userdata.object_name = ''
            self.userdata.object_index = 0

            # Activate the server
            smach.StateMachine.add('ActivateASR',
                    ActivateASR(GRAMMAR_NAME),
                    transitions={'succeeded': 'ask_restaurant_order', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Ask Order
            smach.StateMachine.add(
                'ask_restaurant_order',
                text_to_say("What would you like to order?"),
                transitions={'succeeded': 'ReadASR', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Read from server
            smach.StateMachine.add('ReadASR',
                    ReadASR(),
                    transitions={'succeeded': 'process_restaurant_order', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            # Listen Order
#             smach.StateMachine.add(
#                 'listen_restaurant_order',
#                 ListenToSM(GRAMMAR_NAME),
#                 transitions={'succeeded': 'process_restaurant_order', 'aborted': 'aborted', 
#                 'preempted': 'preempted'}) 
#             
            # Process order
            smach.StateMachine.add(
                'process_restaurant_order',
                process_restaurant_order(),
                transitions={'succeeded': 'ActivateASR_yesno', 'aborted': 'aborted'})
              
            # Load grammar yes/no
            smach.StateMachine.add(
                'ActivateASR_yesno',
                ActivateASR("robocup/yes_no"),
                transitions={'succeeded': 'confirm_restaurant_order', 'aborted': 'aborted', 'preempted': 'preempted'})                    
            
            
            # Confirm Order
            smach.StateMachine.add(
                'confirm_restaurant_order',
                acknowledgment(type_movement="yes"),
                transitions={'succeeded': 'yesno_restaurant_order', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Wait for yes or no
            smach.StateMachine.add(
                'yesno_restaurant_order',
                SayYesOrNoSM(),
                transitions={'succeeded': 'ok_im_going', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
                      
            # Ask for repeat the order
            smach.StateMachine.add(
                'repeat_restaurant_order',
                acknowledgment(type_movement="no",tts_text="Excuse me, I don't understand you. Can you repeat your order?"),
                transitions={'succeeded': 'ReadASR', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Confirm Order
            smach.StateMachine.add(
                'ok_im_going',
                text_to_say("OK i'm going to service"),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            

