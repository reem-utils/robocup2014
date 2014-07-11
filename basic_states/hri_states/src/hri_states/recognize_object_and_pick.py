#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 05/07/2014

@author:  Cristina De Saint Germain
'''

import rospy
import smach
import math
from speech_states.say import text_to_say
from object_grasping_states.pick_object_sm import pick_object_sm
from object_states.recognize_object import recognize_object
from geometry_msgs.msg import PoseStamped
from manipulation_states.play_motion_sm import play_motion_sm

class process_pick_location(smach.State):
    """"
        Getting the POI name of the "Place" location.
    """
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['succeeded', 'aborted', 'preempted'], 
                             input_keys = ['object_position'], 
                             output_keys = ['object_position'])
    def execute(self, userdata):
        if (userdata.object_position.pose.pose.position.z == 0.476659206266) or (math.isnan(userdata.object_position.pose.pose.orientation.x)):    
            return 'aborted'
        else:

            p = PoseStamped()
            p.header.frame_id = userdata.object_position.header.frame_id
            p.pose.position.x = userdata.object_position.pose.pose.position.x
            p.pose.position.z = userdata.object_position.pose.pose.position.z + 0.1
            p.pose.orientation.w = userdata.object_position.pose.pose.orientation.w
            userdata.object_position = p

            return 'succeeded'

    
    
class RecObjectAndPick(smach.StateMachine):
    """
    Executes a SM that tries to recognize an object and if the robot recognize it, tries to pick.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    Input keys:
        object_name: List or string with the object that want to recognize
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'fail_grasp', 'fail_recognize'],
                                     input_keys=['object_name'], 
                                     output_keys=['object_position','object_detected_name'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.nav_to_poi_name=''
            self.userdata.tts_lang = ''
            self.userdata.tts_wait_before_speak = ''
            self.userdata.tts_text = ''
            
            # Home position
            smach.StateMachine.add(
                'home_position',
                play_motion_sm('home'),
                transitions={'succeeded': 'say_start_obj_recognition', 'aborted': 'home_position', #TODO: Change aborted to try again
                'preempted': 'preempted'}) 
            
            # Say start object recognition
            smach.StateMachine.add(
                 'say_start_obj_recognition',
                 text_to_say("I'm going to start the Object recognition process.", wait=False),
                 transitions={'succeeded': 'object_recognition', 'aborted': 'object_recognition'}) 
             
            # Do object_recognition 
            smach.StateMachine.add(
                'object_recognition',
                recognize_object(),
                transitions={'succeeded': 'process_object_recognition', 'aborted': 'fail_recognize', 
                'preempted': 'preempted'}) 
   
            # Process the objects recognized
            smach.StateMachine.add(
                'process_object_recognition',
                process_pick_location(),
                transitions={'succeeded': 'say_grasp_object', 'aborted': 'fail_recognize', 
                'preempted': 'preempted'}) 

            # Say grasp object
            smach.StateMachine.add(
                 'say_grasp_object',
                 text_to_say("I'm going to grasp the object", wait=False),
                 transitions={'succeeded': 'grasp_object', 'aborted': 'aborted'})
            
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                pick_object_sm(),
                transitions={'succeeded': 'succeeded', 'aborted': 'fail_grasp', #TODO: Change aborted to try again
                'preempted': 'preempted'}) 
 
def main():
    rospy.init_node('Rec_Pick_node')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
 
    with sm:
        sm.userdata.object_name = ['Pringles', 'Barritas']
        
        smach.StateMachine.add('Rec_Pick',
                            RecObjectAndPick(),
                            transitions={
                                         'succeeded': 'succeeded', 
                                         'aborted': 'aborted'})
  
    sm.execute()
 
    rospy.spin()
 
if __name__ == '__main__':
    main()                 