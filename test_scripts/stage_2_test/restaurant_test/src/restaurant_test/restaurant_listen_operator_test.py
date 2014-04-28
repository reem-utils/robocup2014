#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
16 Apr 2014

@author: Roger Boldu Busquets
"""

import rospy
import smach
import smach_ros

from restaurant.restaurant_listen_operator import ListenOperator
from navigation_states.nav_to_poi import nav_to_poi

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


def main():
    rospy.init_node('restaurant_listen_operator_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:
        
        smach.StateMachine.add(
            'listen_operator',
            ListenOperator(),
            transitions={'succeeded': 'rospi_go_to_poi','aborted': 'aborted'})
        
        sm.userdata.nav_to_poi_name='drinks'
        smach.StateMachine.add(
            'rospi_go_to_poi',
            nav_to_poi(),
            transitions={'succeeded': 'listen_operator','aborted': 'aborted'})


        



    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant_listen_operator_Test', sm, '/restaurant_guide_listen')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()