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
from restaurant.restaurant_guide_phase import restaurantGuide
from restaurant.restaurant_navigation import RestaurantNavigation
from restaurant.restaurant_order_phase import RestaurantOrder

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


def main():
    rospy.init_node('restaurant_listen_operator_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:
        
        smach.StateMachine.add(
                'ask_order',
                RestaurantOrder(),
                transitions={'succeeded': 'navigation_phase', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Navigation Phase
        smach.StateMachine.add(
                'navigation_phase',
                RestaurantNavigation(),
                transitions={'succeeded': 'leaving_ordering', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            # Leaving the arena  
        smach.StateMachine.add(
                'leaving_ordering',
                nav_to_poi('ordering'),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 



    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant_listen_operator_Test', sm, '/restaurant_guide_listen')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()