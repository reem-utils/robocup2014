#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú Busquets
@email: roger.boldu@gmail.com
"""


import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from speech_states.say import text_to_say

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


TIME_LIVE_AVOID = 5 # maxim time that it will be in this machine 


# Class to decide if i'm bloquet
class blocked(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['time_stamp_init','standard_error'], 
            output_keys=['nav_to_poi_name','standard_error'])

    def execute(self,userdata):
        time= rospy.get_time()
        print str(userdata.time_stamp_init)
        time2=time - userdata.time_stamp_init
        
        if (time2>TIME_LIVE_AVOID):
            userdata.standard_error='Avoid that Time live'+ " + " +userdata.standard_error
            return 'aborted'
        else :
            userdata.nav_to_poi_name='avoid_that'
            return 'succeeded'    
                

# It prepare de Avoid test, get time. and set the poi 
class prepare_Avoid(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=['nav_to_poi_name','standard_error','time_stamp_init'])

    def execute(self,userdata):
        userdata.nav_to_poi_name='avoid_that'
        userdata.time_stamp_init=rospy.get_time()
        return 'succeeded'



class Avoid_That(smach.StateMachine):
    """
    Executes a SM that does the robot stage_1.
    It moves the robot to a POI, maybe it will find a obstacle and thre robot have to Avoid.

    pass the inspection (now we have a dummy state that only waits 5 secs)
    and go to the exit door. We assume that the exit door will be closed.


    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No imput keys.
    Output keys = standard error
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'], output_keys=['standard_error'])
        with self:

            # We prepare the information to go to the init door
            smach.StateMachine.add(
                'prepare_Avoid',
                prepare_Avoid(),
                transitions={'succeeded': 'say_go_to_poi', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
            
            # Announce going to a place
            smach.StateMachine.add(
                    'say_go_to_poi',
                    text_to_say(text="I'm going to my destiny"),
                    transitions={'succeeded': 'go_to_poi'})

            # Go to the POI
            smach.StateMachine.add(
                'go_to_poi',
                nav_to_poi(),
                transitions={'succeeded': 'say_get_to_poi', 'aborted': 'aborted', 
                'preempted': 'preempted'})    
            
            # Announce arriving to a place
            smach.StateMachine.add(
                    'say_get_to_poi',
                    text_to_say(text="I've arrived to my destiny"),
                    transitions={'succeeded': 'succeeded'})

            # If its time to live, will return aborted, else it will tri again sending de same goal
            #smach.StateMachine.add(
            #    'blocked', 
            #   blocked(),
            #  transitions={'succeeded': 'go_to_poi', 'aborted': 'aborted', 
            # 'preempted': 'preempted'}) 

