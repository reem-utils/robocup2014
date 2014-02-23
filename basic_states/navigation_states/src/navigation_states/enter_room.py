#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina
22 Feb 2014

"""


import rospy
import smach
import math

from navigation_states.nav_to_coord import nav_to_coord
from navigation_states.nav_to_poi import nav_to_poi
from navigation_states.get_current_robot_pose import get_current_robot_pose
from sensor_msgs.msg import LaserScan
#from manipulation_states.play_motion import play_motion
from speech_states.say import text_to_say
from util_states.topic_reader import TopicReaderState

class prepare_move_base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['current_robot_pose','nav_to_coord_goal'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):

        distance = 3
        userdata.nav_to_coord_goal=[userdata.nav_to_coord_goal[0], 
                                    userdata.nav_to_coord_goal[1]-distance, 
                                    userdata.nav_to_coord_goal[2]]


        return 'succeeded'

class prepare_play_motion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=[],
                                output_keys=['manip_motion_to_play','manip_time_to_play'])

    def execute(self, userdata):

        userdata.manip_motion_to_play = 'home'
        userdata.manip_time_to_play = 10

        return 'succeeded'

 
class check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'door_too_far'],
         output_keys=['standard_error'])

    def execute(self, userdata):
        distance = 1.5
        door_position = -1
        #rospy.sleep(5)

        message = rospy.wait_for_message('/scan_filtered', LaserScan, 60)
        
        length_ranges = len(message.ranges)
        alpha = math.atan((0.10/2)/distance)
        n_elem = int(math.ceil(length_ranges*alpha/(2*message.angle_max)))  # Num elements from the 0 angle to the left or right
        middle = length_ranges/2
        cut = [x for x in message.ranges[middle-n_elem:middle+n_elem] if x > 0.01]
        minimum = min(cut)
        if door_position == -1:
            if message.ranges[middle] <= 3.0:
                door_position = message.ranges[middle]
            else:
                print 'to far'
                return 'succeeded'

        if (minimum >= distance+door_position):
            return 'succeeded'
        rospy.loginfo("Distance in front of the robot is too small: " + str(distance+door_position) + ". Minimum distance: " + str(minimum))
        userdata.standard_error = "get_current_robot_pose : Time_out getting /scan_filtered"
        return 'aborted'
                        

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

    WIDTH = 0.10  # Width in meters to look forward.
    READ_TIMEOUT = 15
    MAX_DIST_TO_DOOR = 3.0

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['nav_to_coord_goal'],
                                output_keys=['standard_error'])

        with self:
 
            # Check the distance between the robot and the door
            
            # Check door state
            smach.StateMachine.add('check_can_pass',
                   check(),
                   transitions={'succeeded': 'get_actual_pos',
                                'aborted': 'check_can_pass',
                                'door_too_far': 'check_can_pass'})

            # Robot are too far from door
            self.userdata.tts_text = "I'm too far from the door."
            self.userdata.tts_wait_before_speaking = 0

            smach.StateMachine.add(
                'say_too_far_from_door',
                text_to_say(),
                transitions={'succeeded': 'check_can_pass', 'aborted': 'check_can_pass'})
                        
        
            # Robot arms home position
    #        smach.StateMachine.add(
    #            'prepare_home_position',
    #            prepare_play_motion(),
    #            transitions={'succeeded': 'get_actual_pos', 'aborted': 'aborted', 'preempted': 'preempted'})

            # Robot arms home position
    #        smach.StateMachine.add(
    #            'home_position',
    #            play_motion(),
    #            transitions={'succeeded': 'get_actual_pos', 'aborted': 'aborted', 'preempted': 'preempted'})
          
            # Calculate the actual position
            smach.StateMachine.add(
                'get_actual_pos',
                get_current_robot_pose(),
                transitions={'succeeded': 'prepare_move_base', 'aborted': 'aborted', 'preempted': 'succeeded'})

            # Prepare the new goal
            smach.StateMachine.add(
                'prepare_move_base',
                prepare_move_base(),
                transitions={'succeeded': 'enter_room', 'aborted': 'aborted', 'preempted':'preempted'})

            # Enter room
            smach.StateMachine.add(
                'enter_room',
                nav_to_coord(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'preempted'})
                

