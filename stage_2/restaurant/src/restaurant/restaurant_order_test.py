#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
16 Apr 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros

from restaurant_order_phase import RestaurantOrder

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


def main():
    rospy.init_node('restaurant_order_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:
        smach.StateMachine.add(
            'restaurant_order',
            RestaurantOrder(),
            transitions={'succeeded': 'succeeded','aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant_order_introspection', sm, '/ROI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()