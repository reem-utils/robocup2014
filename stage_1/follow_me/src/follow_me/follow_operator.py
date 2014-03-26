#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###

import rospy
import smach

from track_operator import TrackOperator
from smach_ros import ServiceState, SimpleActionState
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from actionlib import SimpleActionClient


# its the traker learn person...
class LearnPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
    def execute(self, userdata):
            rospy.sleep(3)
            rospy.loginfo("i'm in dummy learning face")
            return 'succeeded'

class fixHeadPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
    def execute(self, userdata):
                head_goal = PointHeadGoal()
                head_goal.target.header.frame_id = "base_link"
                head_goal.target.point.x = 1.0
                head_goal.target.point.y = 0.0
                head_goal.target.point.z = 1.65
                head_goal.pointing_frame = "stereo_link"
                head_goal.pointing_axis.x = 1.0
                head_goal.pointing_axis.y = 0.0
                head_goal.pointing_axis.z = 0.0
                head_goal.max_velocity = 1.5
                head_goal.min_duration.secs = 1.5

                client = SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
                client.wait_for_server(rospy.Duration(5.0))

                client.send_goal(head_goal)
                return 'succeeded'


class FollowOperator(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, distToHuman=0.9):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=["in_learn_person"])

        with self:
#TODO i don't know if it's the correct form to stop de face traking
            smach.StateMachine.add('DISABLE_FACE_TRACKING',
                                       ServiceState('/personServer/faceTracking/stop'),
                                       transitions={'succeeded': 'FIX_HEAD_POSITION'})


            smach.StateMachine.add('FIX_HEAD_POSITION',
                                   fixHeadPosition(),
                                  transitions={'succeeded': "LEARN_PERSON",
                                               'preempted': "LEARN_PERSON",
                                            'aborted': "LEARN_PERSON"})

            smach.StateMachine.add('LEARN_PERSON',
                                   LearnPerson(),
                                   transitions={'succeeded': 'TRACK_OPERATOR',
                                                'aborted': 'aborted'},
                                   remapping={'out_personTrackingData': 'out_personTrackingData',
                                              "in_learn_person": "in_learn_person"})

            smach.StateMachine.add('TRACK_OPERATOR',
                                   TrackOperator(distToHuman),
                                   remapping={'in_personTrackingData': 'out_personTrackingData'},
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})