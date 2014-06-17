#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
27/03/2014

@author:Roger Boldu Busquets
"""

import rospy
import smach
import smach_ros

from navigation_states.nav_lateral import nav_lateral

def main():
    rospy.init_node('nav_to_poi_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:

        sm.userdata.nav_to_poi_name = None
        
        smach.StateMachine.add(
            'nav_lateral_test',
            nav_lateral(0.5),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted','preempted':'preempted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'nav_lateral_introspection', sm, '/nav_poi_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()