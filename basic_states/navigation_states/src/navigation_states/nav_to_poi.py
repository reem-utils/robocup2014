#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22 Febreary 12:00:00 2013

@author: Roger Boldu
"""


import rospy
#import copy
import smach
from navigation_states.nav_to_coord import nav_to_coord
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random


# in dis state we will transform de pois to coord
class translate_coord(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['nav_to_poi_goal'], 
         output_keys=['nav_to_coord_goal','standard_error'])

    def execute(self, userdata):
      
         #locationName have the poi that we are looking for
        locationName=userdata.nav_to_poi_goal
        foundLocation = False
        #important to do add the .yalm beefor
        pois = rospy.get_param("/mmap/poi/submap_0") # todo need de mmap
       
        for key, value in pois.iteritems():
         
            if value[1] == locationName:
                userdata.nav_to_coord_goal = [value[2], value[3], value[4]]
                foundLocation = True  
                break

        if foundLocation:
            userdata.standard_error='OK'
            return 'succeeded'
        else :
            userdata.standard_error='Poi not found'
            print FAIL +'POI NOT FOUND'+ENDC # todo change to loginfo
            return 'aborted'


class nav_to_poi(smach.StateMachine):
    """
    Executes a SM that does not much. Transitions 10 times
    randomly transitioning to succeeded or to aborted.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    Input keys: This state machin will recive a string which will contain the poi information
    No output keys.
    No io_keys.

  
    """
    def __init__(self):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
        						input_keys=['nav_to_poi_goal'],
        						output_keys=['standard_error'])


        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.standard_error=''
            self.userdata.nav_to_coord_goal=[0.0,0.0,0.0]
            smach.StateMachine.add(
               'translate_coord',
               translate_coord(),
               transitions={'succeeded': 'move_to_coord', 'aborted': 'aborted', 'preempted': 'aborted'})
                

            smach.StateMachine.add(
             'move_to_coord',
               nav_to_coord(),
               transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                

#             




