#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 25/06/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy
from std_msgs.msg import Bool

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

topic_name='/emergency_stop_state'

class read_emergency_button(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['pressed'], 
                             output_keys=['pressed'])
        self.subs=rospy.Subscriber(topic_name, Bool, self.callback_topic, queue_size=1)
        self.push_emer = None

    def execute(self, userdata):
        if self.push_emer != None:
            userdata.pressed = self.push_emer
            return 'succeeded'
        else: 
            rospy.logerr("There was no /emergency_stop_state data, are we in REEMH32?")
            return 'aborted'
        
    def callback_topic(self,data):
        #If Data is not empty
        self.push_emer = data.data


class emergency_button(smach.StateMachine): 
    """
    Executes a SM that subscribe in a topic and return if the emergency button is pressed.
    
    Required parameters : 
    No parameters.

    No input keys.       
    output keys:
        pressed: Return true if pressed, otherwise returns false. 
    No io_keys.

    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['pressed'], 
                                 output_keys=['pressed'])
        
        with self:
                 
            smach.StateMachine.add(
                                'Read_Topic',
                                read_emergency_button(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
      

