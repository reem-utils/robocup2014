#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
16 Apr 2014

@author: Roger Boldu Busquets
"""

import rospy
import smach
import smach_ros

from restaurant.restaurant_guide_phase import restaurantGuide
from navigation_states.nav_to_poi import nav_to_poi

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


def main():
    rospy.init_node('restaurant_guide_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:

        smach.StateMachine.add(
            'restaurant_guide_test',
            restaurantGuide(),
            transitions={'succeeded': 'succeeded','aborted': 'aborted'})


    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant_guide_Test', sm, '/restaurant_guide')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()