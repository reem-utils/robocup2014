#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
15 Apr 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
import actionlib
import smach_ros

from object_grasping_states.recognize_object import recognize_object

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


# you can chose the name
class prepare_recognize_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=[], 
                             output_keys=['object_name','standard_error'])
        

    def execute(self, userdata):
        userdata.object_name=""
        userdata.object_name=str(raw_input('Object Name: '))
        userdata.standard_error="ok"
        return 'succeeded'   

class recognize_object_print(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['objectd'],output_keys=[])

    def execute(self, userdata):
        
        if userdata.objectd:
            rospy.loginfo (str(userdata.objectd.object_id))
            rospy.loginfo(str(userdata.objectd.object_name))
            rospy.loginfo(str(userdata.objectd.confidence))              
            rospy.loginfo (str(userdata.objectd.position.x))
            rospy.loginfo (str(userdata.objectd.position.y))
            rospy.loginfo (str(userdata.objectd.position.z))   

        return 'succeeded'
    
    
def main():
    rospy.init_node('recognize_object_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
       
        smach.StateMachine.add(
            'prepare_object_recognize',
            prepare_recognize_object(),
            transitions={'succeeded': 'object_recognition','aborted': 'aborted'})

        smach.StateMachine.add(
            'object_recognition',
            recognize_object(),
            transitions={'succeeded': 'object_print','aborted': 'aborted'})
        
        smach.StateMachine.add(
            'object_print',
            recognize_object_print(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'detect_object_introspection', sm, '/DOI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
