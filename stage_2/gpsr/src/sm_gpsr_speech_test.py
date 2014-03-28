#! /usr/bin/env python

import roslib
roslib.load_manifest('gpsr')
import smach
from smach_ros import SimpleActionState
import rospy

from pal_smach_utils.utils.global_common import succeeded, preempted, aborted, transform_pose
from pal_smach_utils.speech.sm_listen_orders import ListenOrders  # check if the SM is correctly defined

#seat, drink, food, snack, cleaning_stuff, table, shelf, utensil, appliance, seating


class gpsrOrders(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
        self.referee_position = None
        with self:          

            smach.StateMachine.add(
                    'LISTEN_ORDER',
                    ListenOrders(GRAMMAR_NAME='robocup/seat'),
                    transitions={succeeded: 'LISTEN_ORDER', aborted: 'LISTEN_ORDER'})


def main():
    rospy.init_node('gpsr_speech_test')

    sm = gpsrOrders()
    sm.execute()

    rospy.spin()
  

if __name__ == '__main__':
    main()

