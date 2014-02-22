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
from navigation_states.get_current_robot_pose import get_current_robot_pose
from sensor_msgs.msg import LaserScan
#from manipulation_states.play_motion import play_motion
from speech_states.say import text_to_say

class prepare_move_base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['current_robot_pose'],
                                output_keys=['nav_to_coord_goal'])


    def execute(self, userdata):

        distance = 1.5
        userdata.nav_to_coord_goal = [current_robot_pose.x + distance, current_robot_pose.y, current_robot_pose.yaw]

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
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
 
            # Check the distance between the robot and the door
            @smach.cb_interface(outcomes=[succeeded, aborted, preempted, 'door_too_far'])
            def check_range(userdata, message):
                length_ranges = len(message.ranges)
                alpha = math.atan((self.WIDTH/2)/distance)
                n_elem = int(math.ceil(length_ranges*alpha/(2*message.angle_max)))  # Num elements from the 0 angle to the left or right
                middle = length_ranges/2
                cut = [x for x in message.ranges[middle-n_elem:middle+n_elem] if x > 0.01]
                print "cut: " + str(cut)
                minimum = min(cut)
                if self.door_position == -1:
                    if message.ranges[middle] <= self.MAX_DIST_TO_DOOR:
                        self.door_position = message.ranges[middle]
                    else:
                        return 'door_too_far'
                if (minimum >= distance+self.door_position):
                    return succeeded
                rospy.loginfo("Distance in front of the robot is too small: " + str(distance+self.door_position) + ". Minimum distance: " + str(minimum))
                return aborted

            # Check door state
            smach.StateMachine.add('check_can_pass',
                   TopicReaderState(topic_name='scan_filtered',
                                    msg_type=LaserScan,
                                    timeout=self.READ_TIMEOUT,
                                    callback=check_range,
                                    outcomes=[succeeded, aborted, preempted, 'door_too_far']),
                   remapping={'message': 'range_readings'},
                   transitions={succeeded: 'home_position',
                                aborted: 'check_can_pass',
                                'door_too_far': 'say_too_far_from_door'})

            # Robot are too far from door
            sm.userdata.tts_text = "I'm too far from the door."
            sm.userdata.tts_wait_before_speaking = 0

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
                

