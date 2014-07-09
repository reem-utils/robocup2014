#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10 July 00:20:00 2014

@author: Chang Long Zhu Jin
@email: changlongzj@gmail.com

"""

import rospy
import smach
from navigation_states.nav_to_coord import nav_to_coord
from geometry_msgs.msg import Pose
from util_states.math_utils import *

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

DISTANCE_TO_HUMAN = 0.4

class prepareData(smach.State):
    
    def __init__(self, x, y, yaw, distance_to_human):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_coord_goal'], output_keys=['nav_to_coord_goal'])
        self.init_x = x
        self.init_y = y
        self.init_yaw = yaw
        self.distance_to_human = distance_to_human
        
    def execute(self, userdata):
                  
        #Priority in init
        userdata.nav_to_coord_goal[0] = self.init_x if self.init_x else userdata.nav_to_coord_goal[0]
        userdata.nav_to_coord_goal[1] = self.init_y if self.init_y else userdata.nav_to_coord_goal[1]
        userdata.nav_to_coord_goal[2] = self.init_yaw if self.init_yaw else userdata.nav_to_coord_goal[2]
        
        if None in userdata.nav_to_coord_goal:
            rospy.logerr("No Goal to Send... Error")
            return 'aborted'
         
        #Calculating vectors for the position indicated
        new_pose = Pose()
        
        new_pose.position.x = userdata.nav_to_coord_goal[0]
        new_pose.position.y = userdata.nav_to_coord_goal[1]
        
        unit_vector = normalize_vector(new_pose.position)
        
        position_distance = vector_magnitude(new_pose.position)
        
        rospy.loginfo(" Position data from Reem to person:")
        rospy.loginfo(" Position vector : " + str(new_pose.position))
        rospy.loginfo(" Unit position vector : " + str(unit_vector))
        rospy.loginfo(" Position vector distance : " + str(position_distance))

        """
        If person is closer than the distance given, we wont move but we might rotate.
        We want that if the person comes closer, the robot stays in the place.
        Thats why we make desired distance zero if person too close.
        """

        distance_des = 0.0
        
        if position_distance >= self.distance_to_human: 
            distance_des = position_distance - self.distance_to_human

        else:
            rospy.loginfo(OKGREEN+" Person too close => not moving, just rotate"+ENDC)

        #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
        
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

        rospy.loginfo(' Final robot movement data:')
        rospy.loginfo(' Distance from robot center to person : ' + str(position_distance))
        rospy.loginfo(' Person and Reem wanted distance (distance to human) : ' + str(self.distance_to_human))
        rospy.loginfo(' Distance that REEM will move towards the person : ' + str(distance_des))
        rospy.loginfo(' Degrees that REEM will rotate : ' + str(alfa_degree))


        userdata.nav_to_coord_goal = [dist_vector.x, dist_vector.y, alfa]
        
         
        return 'succeeded'
    

class nav_to_coord_dist(smach.StateMachine):
    """
    This state machine receive the name of the point and go there. 

    Required parameters:
        No required parameters.

    Optional parameters:
        @param x: 
        @param y: 
        @param yaw: 
        @param distance_to_human: 

    Input keys: 
        @key nav_to_coord_goal: [x, y, yaw]
    Output keys:
        @key standard_error: String that show what kind of error could be happened
    
    No io_keys.

  
    """
    def __init__(self, x = None, y = None, yaw = None, distance_to_human = DISTANCE_TO_HUMAN):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                input_keys=['nav_to_coord_goal'],
                                output_keys=['standard_error'])


        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.standard_error=''

            smach.StateMachine.add('PrepareData',
               prepareData(x, y, yaw, distance_to_human),
               transitions={'succeeded':'move_to_coord', 
                            'aborted':'aborted'})

            smach.StateMachine.add(
               'move_to_coord',
               nav_to_coord(),
               transitions={'succeeded': 'succeeded', 
                            'aborted': 'aborted', 
                            'preempted': 'preempted'})
                
             

def main():
    rospy.loginfo('Go POi Node')
    rospy.init_node('go_poi')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:      
        sm.userdata.nav_to_coord_goal = [0, 0, 0]
        smach.StateMachine.add(
            'Go_to_dist',
            nav_to_coord_dist(x = 1.0, y = 2.0, yaw = 1),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()



