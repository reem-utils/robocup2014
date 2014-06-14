#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
27/03/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros

from navigation_states.nav_cancel_poi import cancel_nav

def main():
    rospy.init_node('nav_cancel_poi')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:
        
        smach.StateMachine.add(
            'nav_cancel_poi',cancel_nav(),
            transitions={'succeeded': 'succeeded','preempted':'preempted',
                         'aborted' : 'aborted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'nav_cancel_introspection', sm, '/nav_cancel_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()