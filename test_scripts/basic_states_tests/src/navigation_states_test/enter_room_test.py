#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
27/03/2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros

from navigation_states.enter_room import EnterRoomSM

def main():
    rospy.init_node('enter_room_test')
    

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    with sm:

        sm.userdata.nav_to_poi_name = None
        
        smach.StateMachine.add(
            'enter_room_test',
            EnterRoomSM("entrance"),
            transitions={'succeeded': 'succeeded','aborted' : 'aborted'})
        
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'enter_room_introspection', sm, '/enterRoom_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
