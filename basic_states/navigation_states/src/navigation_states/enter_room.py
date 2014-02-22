#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina
22 Feb 2014

"""


import rospy
import smach

from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from sensor_msgs.msg import LaserScan
#Falta import del speech
#Falta import del get_current_pose


class prepare_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['current_robot_pose'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):

        distance = 1.5
        userdata.nav_to_coord_goal = [current_robot_pose.x + distance, current_robot_pose.y, current_robot_pose.yaw]

        return 'succeeded'

           

class EnterRoomSM(smach.StateMachine):
    """
    Executes a SM that enter a room. 
    Look for a door, wait if not open and enter the room.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
 
            # We need to say which door want to go
            self.userdata.nav_to_poi_goal = 'doorIn' # TODO: We need a state to prepare info and it will be global

            # Go to the door
            smach.StateMachine.add(
                'go_to_door',
                nav_to_poi(),
                transitions={'succeeded': 'move_to_intermediate_poi', 'aborted': 'aborted', 'preempted': 'succeeded'})

            # We need look to the door?

            # Look for a door
            smach.StateMachine.add('CHECK_CAN_PASS',
                TopicReaderState(topic_name='scan_filtered',
                                msg_type=LaserScan,
                                timeout=self.READ_TIMEOUT,
                                callback=check_range,
                                outcomes=[succeeded, aborted, preempted, 'door_too_far']),
                                remapping={'message': 'range_readings'},
                                transitions={succeeded: 'ENTER_ROOM',
                                            aborted: 'CHECK_CAN_PASS',
                                            'door_too_far': 'SAY_TOO_FAR_FROM_DOOR'})

            # Check if open
            smach.StateMachine.add(
                'check_if_open',
                DummyStateMachine(),
                transitions={'succeeded': 'get_actual_pos', 'aborted': 'say_cant_open', 'preempted': 'say_cant_open'})

            # Ask for help
            smach.StateMachine.add(
                'say_cant_open', 
                SpeakActionState(text="I can't open the door. Can you please open it for me?"),
                transitions={succeeded: 'check_if_open', aborted: 'check_if_open'})

            # Calculate the actual position
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pos(),
                transitions={'succeeded': 'move_to_intermediate_poi', 'aborted': 'aborted', 'preempted': 'succeeded'})

            # Prepare the new goal
            smach.StateMachine.add(
                'prepare_goal',
                prepare_goal(),
                transitions={'succeeded': 'enter_room', 'aborted': 'aborted', 'preempted':'preempted'}

            # Could prepare arms?

            # Enter room
            smach.StateMachine.add(
                'enter_room',
                nav_to_coord(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                

