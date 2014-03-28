#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue March 23 12:00:00 2013

@author: Chang Long ZHu Jin
@email: changlongzj@gmail.com
"""

import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState
from  object_grasping_states.detect_object_sm import detect_object
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'




# you can chose the name
class prepare_detect_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=[], 
                             output_keys=['name','standard_error'])
        

    def execute(self, userdata):
        userdata.name=str(raw_input('Object Name :'))
        userdata.standard_error="ok"
        return 'succeeded'   
    
class recognize_Object_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of aborted recognize object')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'
    
class recognize_Object_print(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],input_keys=['objectm'],output_keys=['objectm'])

    def execute(self, userdata):
        
        if userdata.objectm :
            rospy.loginfo(str(userdata.objectm.object_name))
            rospy.loginfo(str(userdata.objectm.confidence))              
            rospy.loginfo (str(userdata.objectm.position))

        return 'succeeded'
    
    
def main():
    rospy.init_node('recognize_obect_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # smach.StateMachine.add(
        #     'prepare_object',
        #     prepare_detect_object(),
        #     transitions={'succeeded': 'detect_object','aborted' : 'aborted_info'})
        # it call the learn state
        smach.StateMachine.add(
            'detect_object',
            detect_object(),
            transitions={'succeeded': 'detect_print','aborted' : 'aborted_info'})
        # it prints the standard error
        smach.StateMachine.add(
            'aborted_info',
            recognize_Object_print(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})
         # it prints the standard error
        
        sm.userdata.object=None
        
        smach.StateMachine.add(
            'detect_print',
            recognize_Object_print(),
            transitions={'succeeded': 'aborted_info', 'aborted':'aborted_info'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'detect_object_introspection', sm, '/DFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
