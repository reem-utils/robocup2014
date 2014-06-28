#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
27/03/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros

from navigation_states.nav_to_poi import nav_to_poi

def main():
    rospy.init_node('nav_to_poi_test')
    

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:

        sm.userdata.nav_to_poi_name = None
        
        smach.StateMachine.add(
            'nav_poi_test',
            nav_to_poi("kitchen"),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'nav_poi_introspection', sm, '/nav_poi_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
