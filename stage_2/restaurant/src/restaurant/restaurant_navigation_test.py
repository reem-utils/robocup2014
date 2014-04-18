#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
16 Apr 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import smach_ros

from restaurant_navigation import RestaurantNavigation

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


# you can chose the name
class prepare_object_array(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=['object_array'], 
                             output_keys=['object_array'])
        

    def execute(self, userdata):
        userdata.object_array[0]=['ObjectA', 'beverage', 'delivery1']
        userdata.object_array[1]=['ObjectB', 'snack', 'delivery2']
        userdata.object_array[2]=['ObjectC', 'beverage', 'delivery3']
        
        return 'succeeded'   

def main():
    rospy.init_node('restaurant_navigation_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
       
        sm.userdata.object_array = 3*[3*[0]]
       
        smach.StateMachine.add(
            'prepare_object_array',
            prepare_object_array(),
            transitions={'succeeded': 'restaurant_navigation','aborted': 'aborted'})

        smach.StateMachine.add(
            'restaurant_navigation',
            RestaurantNavigation(),
            transitions={'succeeded': 'succeeded','aborted': 'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'restaurant_navigation_introspection', sm, '/RNI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()