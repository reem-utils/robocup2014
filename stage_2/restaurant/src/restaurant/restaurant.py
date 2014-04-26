#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 6/04/2014

@author: Cristina De Saint Germain
'''

import rospy
import smach
import smach_ros
import actionlib

from restaurant_sm import RestaurantSM

def main():
    rospy.init_node('restaurant')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
        smach.StateMachine.add(
            'RestaurantSM',
            RestaurantSM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted','preempted':'preempted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant', sm, '/restaurant_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
