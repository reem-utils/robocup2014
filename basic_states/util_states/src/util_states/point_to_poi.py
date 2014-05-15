#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22 Febreary 12:00:00 2013

@author: Sergi Xavier Ubach Pallàs
@email: sxubcah@gmail.com

"""

import rospy
import smach
from navigation_states import GetPoseSubscribe

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class prepareData(smach.State):
    
    def __init__(self, poi_name):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['point_to_poi_name'], output_keys=['point_to_poi_name'])
        self.poi_name = poi_name
        
    def execute(self, userdata):
           
        if not self.poi_name and not userdata.point_to_poi_name:
            rospy.logerr("Poi_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.point_to_poi_name = self.poi_name if self.poi_name else userdata.point_to_poi_name   
        
        return 'succeeded'
    

# In this state we will transform de pois to coord
class translate_coord(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['point_to_poi_name'], 
         output_keys=['point_to_coord_goal','standard_error'])

    def execute(self, userdata):
      
        #locationName have the poi that we are looking for
        locationName=userdata.point_to_poi_name
        foundLocation = False
        #important to do add the .yalm before
        pois = rospy.get_param("/mmap/poi/submap_0")
       
        for key, value in pois.iteritems():
         
            if value[1] == locationName:
                userdata.point_to_coord_goal = [value[2], value[3], value[4]]
                foundLocation = True  
                break

        if foundLocation:
            userdata.standard_error='OK'
            return 'succeeded'
        else :
            userdata.standard_error='Poi not found'
            rospy.loginfo( FAIL +'POI NOT FOUND im locking for'+locationName+ENDC)
            return 'aborted'


# In this state we will calculate how to rotate to face the poi
class calculateYaw(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['current_robot_pose', 'point_to_coord_goal'],
         output_keys=['desired_angle'])

    def execute(self, userdata):
        dif_cord.x = userdata.point_to_coord_goal[0] - userdata.current_robot_pose.position.x
        dif_cord.y = userdata.point_to_coord_goal[1] - userdata.current_robot_pose.position.y
        userdata.desired_angle = atan2(dif_cord.y/dif_cord.x)
        return "succeeded"
        
# In this state we will point in front of the robot
class point_to_coord(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['point_to_poi_name'])

    def execute(self, userdata):
        #CHANG make this point forward
        return "succeeded"
        

class point_to_poi(smach.StateMachine):
    """
    This state machine receive the name of the point and points it. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters# todo need de mmap

    Input keys: 
        point_to_poi_name: String that contain the poi information
    Output keys:
        standard_error: String that show what kind of error could be happened
    No io_keys.  
    """
    def __init__(self, poi_name = None):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
        						input_keys=['point_to_poi_name'],
        						output_keys=['standard_error'])


        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.standard_error=''
            self.userdata.point_to_coord_goal=[0.0,0.0,0.0]

            smach.StateMachine.add('PrepareData',
               prepareData(poi_name),
               transitions={'succeeded':'translate_coord', 'aborted':'aborted'})

            # We transform the poi to coordenades
            smach.StateMachine.add('translate_coord',
               translate_coord(),
               transitions={'succeeded': 'get_pose', 'aborted': 'aborted', 'preempted': 'aborted'})

            # We get current coodenades of robot in userdata 'current_robot_pose', 'current_robot_yaw', 'pose_current'
            smach.StateMachine.add('get_pose',
               GetPoseSubscribe(),
               transitions={'succeeded': 'get_yaw', 'aborted': 'aborted', 'preempted': 'aborted'})

            # We get current coodenades of robot in userdata 'current_robot_pose', 'current_robot_yaw', 'pose_current'
            smach.StateMachine.add('get_yaw',
               calculateYaw(), #output ['desired_angle']
               transitions={'succeeded': 'turn', 'aborted': 'aborted', 'preempted': 'aborted'})
                
            # Turn to face poi
            smach.StateMachine.add('turn',
               turn(), #TODO: Roger is making the turn function
               transitions={'succeeded': 'point_to_coord', 'aborted': 'aborted', 'preempted': 'aborted'})
                
            # Point the coordenades
            smach.StateMachine.add('point_to_coord',
               point_to_coord(), #TODO: move_it
               transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                
             




