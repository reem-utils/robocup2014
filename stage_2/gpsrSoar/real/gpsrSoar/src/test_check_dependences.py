#! /usr/bin/env python
# -.- coding: utf-8 -.-

import roslib
roslib.load_manifest('cocktail_party')
import smach
import rospy
import smach_ros

from pal_smach_utils.utils.global_common import succeeded, aborted, preempted
from pal_smach_utils.utils.check_dependences import CheckDependencesState

TOPICS = ["/usersaid"]
ACTIONS = ["/left_arm_controller/joint_trajectory_action", "/right_arm_torso_controller/joint_trajectory_action", "/move_base", "/move_by/move_base", "/move_left_arm", "/move_right_arm_torso", "/face_recognition"]
MAP_LOC = ["bench", "hallway table", "hanger", "umbrella stand", "bar", "side table", "kitchen table", "kitchen counter", "stove", "trash bin", "cupboard", "sink", "fridge", "bed", "dresser", "sideboard", "bedside table", "bookshelf", "pantry", "cabinet", "dinner table", "couch table", "arm chair", "sofa", "tv counter",  "plant", "exit"]


def main():
    rospy.init_node('test_check_dependences')
    sm = smach.StateMachine(outcomes=[succeeded, aborted, preempted])
    with sm:

        smach.StateMachine.add("TEST_CHECK_DEPENDENCES",
            CheckDependencesState(
                    topic_names=TOPICS,
                    service_names=None,
                    action_names=ACTIONS,
                    map_locations=MAP_LOC),
            transitions={succeeded: succeeded, aborted: aborted}
            )

    sis = smach_ros.IntrospectionServer(
        'test_check_dependences_introspection', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
