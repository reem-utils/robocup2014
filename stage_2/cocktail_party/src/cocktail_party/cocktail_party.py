#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 22/03/2014

@author: Cristina De Saint Germain
'''

import rospy
import smach
import smach_ros
import actionlib

from check_dependences import CheckDependencesState
from cocktail_party_sm import CocktailPartySM

def main():
    rospy.init_node('cocktail_party')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:
   
#         smach.StateMachine.add(
#             'CheckDepencences',
#             CheckDependencesState(),
#             transitions={'succeeded': 'CocktailPartySM', 'aborted': 'aborted'}) 
#    
        smach.StateMachine.add(
            'CocktailPartySM',
            CocktailPartySM(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'cocktail_party', sm, '/CP_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
