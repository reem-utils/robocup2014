#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""

import rospy
import smach
import smach_ros

#from smach_ros import SimpleActionState, ServiceState
#from face_states.learn_face import learn_face
from face_states.recognize_face import recognize_face
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'




# you can chose the name
class prepare_recognize_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],
                             input_keys=[], 
                             output_keys=['name','standard_error'])
        

    def execute(self, userdata):
        userdata.name=""
        userdata.name=str(raw_input('Face Name :'))
        userdata.standard_error="ok"
        return 'succeeded'   
    
class recognize_Face_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of aborted recognize Face')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'
    
class recognize_Face_print(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['face'],output_keys=[])

    def execute(self, userdata):
        
        if userdata.face :
            rospy.loginfo(str(userdata.face.name))
            rospy.loginfo(str(userdata.face.confidence))              
            rospy.loginfo (str(userdata.face.x))
            rospy.loginfo (str(userdata.face.y))
            rospy.loginfo (str(userdata.face.width))
            rospy.loginfo (str(userdata.face.height))
            rospy.loginfo (str(userdata.face.eyesLocated))
            rospy.loginfo (str(userdata.face.leftEyeX))
            rospy.loginfo (str(userdata.face.leftEyeY))
            rospy.loginfo (str(userdata.face.rightEyeX))
            rospy.loginfo (str(userdata.face.rightEyeY))
            rospy.loginfo (str(userdata.face.position.x))
            rospy.loginfo (str(userdata.face.position.y))
            rospy.loginfo (str(userdata.face.position.z))
         

        return 'succeeded'
    
    
def main():
    rospy.init_node('recognize_face_test')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        
        
                # it call the learn state
        smach.StateMachine.add(
            'prepare_face_reconize',
            prepare_recognize_face(),
            transitions={'succeeded': 'detect_face','aborted' : 'aborted_info'})
        # it call the learn state
        smach.StateMachine.add(
            'detect_face',
            recognize_face(),
            transitions={'succeeded': 'detect_print','aborted' : 'aborted_info'})
        # it prints the standard error
        smach.StateMachine.add(
            'aborted_info',
            recognize_Face_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})
         # it prints the standard error
        
        sm.userdata.face=None
        
        smach.StateMachine.add(
            'detect_print',
            recognize_Face_print(),
            transitions={'succeeded': 'aborted_info', 'aborted':'aborted_info'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'detect_face_introspection', sm, '/DFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
