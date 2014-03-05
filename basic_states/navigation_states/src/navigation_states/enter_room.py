#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com
@author: Roger Boldu
@email: roger.boldu@gmail.com

22 Feb 2014

"""

import rospy
import smach
import math

from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.get_current_robot_pose import get_current_robot_pose
from sensor_msgs.msg import LaserScan
from manipulation_states.play_motion_sm import play_motion_sm
from speech_states.say_sm import text_to_say
from util_states.topic_reader import topic_reader

class prepare_play_motion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=[],
                                output_keys=['manip_motion_to_play','manip_time_to_play'])

    def execute(self, userdata):

        userdata.manip_motion_to_play = 'home'
        userdata.manip_time_to_play = 10

        return 'succeeded'    

class prepare_say_far(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=[],
                                output_keys=['tts_text','tts_wait_before_speaking'])

    def execute(self, userdata):

        userdata.tts_text = "I'm too far from the door."
        userdata.tts_wait_before_speaking = 0

        return 'succeeded'                       

class prepare_say_open_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=[],
                                output_keys=['tts_text','tts_wait_before_speaking'])

    def execute(self, userdata):

        userdata.tts_text = "Can anyone open the door please?"
        userdata.tts_wait_before_speaking = 0

        return 'succeeded'  

class check_door_status(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'door_too_far'],
         output_keys=['standard_error'])
        self.door_position = -1

    def execute(self, userdata):

     #   rospy.sleep(10) 

        distance = 1.5
        
        WIDTH = 0.10  # Width in meters to look forward.
        READ_TIMEOUT = 15
        MAX_DIST_TO_DOOR = 3.0
        #rospy.sleep(5)

        message = rospy.wait_for_message('/scan_filtered', LaserScan, 60)
        
        # Check the distance between the robot and the door
        length_ranges = len(message.ranges)
        alpha = math.atan((WIDTH/2)/distance)
        n_elem = int(math.ceil(length_ranges*alpha/(2*message.angle_max)))  # Num elements from the 0 angle to the left or right
        middle = length_ranges/2
        cut = [x for x in message.ranges[middle-n_elem:middle+n_elem] if x > 0.01]
        minimum = min(cut)
        if self.door_position == -1:
            if message.ranges[middle] <= MAX_DIST_TO_DOOR:
                self.door_position = message.ranges[middle]
            else:
                return 'door_too_far'

        if (minimum >= distance+self.door_position):
            return 'succeeded'
        rospy.loginfo("Distance in front of the robot is too small: " + str(distance+self.door_position) + ". Minimum distance: " + str(minimum))
        userdata.standard_error = "get_current_robot_pose : Time_out getting /scan_filtered"
        return 'aborted'
                        

class EnterRoomSM(smach.StateMachine):
    """
    Executes a SM that enter a room. 
    Wait if the door isn't open and enter the room.

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input keys:
        nav_to_poi_name: indicates the point we need to reach after detect that the door is open
    Output keys:
        standard_error: String that show what kind of error could be happened
    No io_keys.

    Nothing must be taken into account to use this SM.
    """

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['nav_to_poi_name'],
                    output_keys=['standard_error'])

        with self:
    
            # Check door state
            smach.StateMachine.add('check_can_pass',
                   check_door_status(),
                   transitions={'succeeded': 'prepare_home_position',
                                'aborted': 'prepare_say_open_door',
                                'door_too_far': 'prepare_say_far'})

            # Robot is too far from door
            smach.StateMachine.add(
                'prepare_say_far',
                prepare_say_far(),
                transitions={'succeeded': 'say_too_far_from_door', 'aborted': 'aborted', 'preempted': 'preempted'})

            smach.StateMachine.add(
                'say_too_far_from_door',
                text_to_say(),
                transitions={'succeeded': 'check_can_pass', 'aborted': 'check_can_pass'})
                        
        
            # Robot ask to open the door
            smach.StateMachine.add(
                'prepare_say_open_door',
                prepare_say_open_door(),
                transitions={'succeeded': 'say_open_door', 'aborted': 'aborted', 'preempted': 'preempted'})

            smach.StateMachine.add(
                'say_open_door',
                text_to_say(),
                transitions={'succeeded': 'check_can_pass', 'aborted': 'check_can_pass'})

            # Robot arms home position
            smach.StateMachine.add(
                'prepare_home_position',
                prepare_play_motion(),
                transitions={'succeeded': 'home_position', 'aborted': 'aborted', 'preempted': 'preempted'})

            # Robot arms home position
            smach.StateMachine.add(
                'home_position',
                play_motion_sm(),
                transitions={'succeeded': 'enter_room', 'aborted': 'aborted', 'preempted': 'preempted'})
          
            # We don't need to prepare the state, it takes the input_key directly

            # Go to the poi in the other site of the door
            smach.StateMachine.add(
                'enter_room',
                nav_to_poi(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})


