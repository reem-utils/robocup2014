#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22 Febreary 12:00:00 2013

@author: Roger Boldu
@email: roger.boldu@gmail.com

"""

import rospy
import smach
from navigation_states.nav_to_coord import nav_to_coord

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class prepareData(smach.State):
    
    def __init__(self, poi_name):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_poi_name'], output_keys=['nav_to_poi_name'])
        self.poi_name = poi_name
        
    def execute(self, userdata):
           
        if not self.poi_name and not userdata.nav_to_poi_name:
            rospy.logerr("Poi_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.nav_to_poi_name = self.poi_name if self.poi_name else userdata.nav_to_poi_name   
        
        return 'succeeded'
    

# In this state we will transform de pois to coord
class translate_coord(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['nav_to_poi_name'], 
         output_keys=['nav_to_coord_goal','standard_error'])

    def execute(self, userdata):
      
        #locationName have the poi that we are looking for
        locationName=userdata.nav_to_poi_name
        foundLocation = False
        #important to do add the .yalm before
        pois = rospy.get_param("/mmap/poi/submap_0")
       
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
            rospy.loginfo( FAIL +'POI NOT FOUND im locking for'+locationName+ENDC)
            return 'aborted'


class nav_to_poi(smach.StateMachine):
    """
    This state machine receive the name of the point and go there. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters# todo need de mmap

    Input keys: 
        nav_to_poi_name: String that contain the poi information
    Output keys:
        standard_error: String that show what kind of error could be happened
    No io_keys.

  
    """
    def __init__(self, poi_name = None):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
        						input_keys=['nav_to_poi_name'],
        						output_keys=['standard_error'])


        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.standard_error=''
            self.userdata.nav_to_coord_goal=[0.0,0.0,0.0]

            smach.StateMachine.add('PrepareData',
               prepareData(poi_name),
               transitions={'succeeded':'translate_coord', 'aborted':'aborted'})

            # We transform the poi to coordenates
            smach.StateMachine.add(
               'translate_coord',
               translate_coord(),
               transitions={'succeeded': 'move_to_coord', 'aborted': 'aborted', 'preempted': 'aborted'})
                
            # Go to coordanates
            smach.StateMachine.add(
             'move_to_coord',
               nav_to_coord(),
               transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                
             




