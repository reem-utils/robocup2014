#!/usr/bin/env python

import rospy
import smach

#from global_common import ''preempted'', aborted, preempted, o1, o2, o3, o4
class topic_reader_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             input_keys=['topic_name', 'topic_type', 'topic_time_out'],
                             output_keys=['topic_info', 'standard_error'])

    def execute(self, userdata):
        try:
            _topic_info = rospy.wait_for_message(userdata.topic_name, userdata.topic_type, userdata.topic_time_out)
            userdata.topic_info = _topic_info
            return 'succeeded'
        except rospy.ROSException:
            userdata.standard_error = "get_current_robot_pose : Time_out getting "
            return 'aborted'
        
class topic_reader(smach.StateMachine):
    
    def __init__(self):
        
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['topic_name', 'topic_type', 'topic_time_out'],
                             output_keys=['topic_info', 'standard_error'])
        with self:
            smach.StateMachine.add('Topic_reader_state', 
                                   topic_reader_state(), 
                                   transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})