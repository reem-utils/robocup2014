#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com
@author: Roger Boldu
@email: roger.boldu@gmail.com
@author: Sergi Ubach

23 Feb 2014

"""


import rospy
import smach
import math

from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.get_current_robot_pose import get_current_robot_pose

class prepare_move_base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['current_robot_pose','current_robot_yaw', 'nav_to_coord_goal', 'distance'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):

        distX = (userdata.distance / math.sqrt(math.pow(math.tan(userdata.current_robot_yaw),2) + 1) ) 
        distY = math.tan(userdata.current_robot_yaw) * distX

        print "distX: " + str(distX)
        print "distY: " + str(distY)

        if 0 <= userdata.current_robot_yaw <= math.pi/2:
            userdata.nav_to_coord_goal=[userdata.current_robot_pose.pose.position.x + distX, 
                                        userdata.current_robot_pose.pose.position.y + distY, 
                                        userdata.current_robot_yaw]
        elif math.pi/2 < userdata.current_robot_yaw <= math.pi:
            userdata.nav_to_coord_goal=[userdata.current_robot_pose.pose.position.x - distX, 
                                        userdata.current_robot_pose.pose.position.y + distY, 
                                        userdata.current_robot_yaw]

                                        
        elif -math.pi/2 > userdata.current_robot_yaw >= -math.pi:  
            userdata.nav_to_coord_goal=[userdata.current_robot_pose.pose.position.x - distX, 
                                        userdata.current_robot_pose.pose.position.y - distY, 
                                        userdata.current_robot_yaw]                              
        elif 0 >= userdata.current_robot_yaw >= -math.pi/2:
            userdata.nav_to_coord_goal=[userdata.current_robot_pose.pose.position.x + distX, 
                                        userdata.current_robot_pose.pose.position.y - distY, 
                                        userdata.current_robot_yaw]
 
        print "Robot x: " + str(userdata.current_robot_pose.pose.position.x)
        print "Robot y: " + str(userdata.current_robot_pose.pose.position.y)
        print "Robot new x: " + str(userdata.nav_to_coord_goal[0])
        print "Robot new y: " + str(userdata.nav_to_coord_goal[1])
        print "Robot yaw: " + str(userdata.current_robot_yaw)

        return 'succeeded'

class goForwardSM(smach.StateMachine):
    """
    Executes a SM that go forward respect the actual position. 
    It has a little error with yaw. 

    TODO: It's a beta test

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Inputs keys:
        distance: The meters that the robot go forward

    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['distance'],
                    output_keys=[])

        with self:
            
            # Calculate the actual position
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pose(),
                transitions={'succeeded': 'prepare_move_base', 'aborted': 'aborted', 'preempted': 'succeeded'})

            # Prepare the new goal
            smach.StateMachine.add(
                'prepare_move_base',
                prepare_move_base(),
                transitions={'succeeded': 'go_to_point', 'aborted': 'aborted', 'preempted':'preempted'})

            # Go to the new point
            smach.StateMachine.add(
                'go_to_point',
                nav_to_coord(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                

