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
from face_states.detect_faces import detect_face
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class prepare_detect_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=['minCofidence'])
        
    def execute(self, userdata):
        rospy.loginfo("prepering learn face")
        userdata.minCofidence=90.0 # is the name that it will learn TODO: i don't know normal parameter
        return 'succeeded'
    
class detect_Face_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of aborted detect Face')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'
    
class detect_Face_print(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['faces'],output_keys=[])

    def execute(self, userdata):
        
                for object in userdata.faces.faces:
                    rospy.loginfo( OKGREEN +"Face :==   "
                                   +str(object.name)+"  "+
                                   str(object.confidence) + ENDC)
                                   
                    rospy.loginfo (str(object.x))
                    rospy.loginfo (str(object.y))
                    rospy.loginfo (str(object.width))
                    rospy.loginfo (str(object.height))
                    rospy.loginfo (str(object.eyesLocated))
                    rospy.loginfo (str(object.leftEyeX))
                    rospy.loginfo (str(object.leftEyeY))
                    rospy.loginfo (str(object.rightEyeX))
                    rospy.loginfo (str(object.rightEyeY))
                    rospy.loginfo (str(object.position.x))
                    rospy.loginfo (str(object.position.y))
                    rospy.loginfo (str(object.position.z))
                    
       
                return 'succeeded'
    
    
def main():
    rospy.init_node('detect_face_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # it preper the minConfidence for detect
        smach.StateMachine.add(
            'prepare_detect_face',
            prepare_detect_face(),
            transitions={'succeeded':'detect_face','aborted' : 'aborted','preempted':'preempted'})
        # it call the learn state
        smach.StateMachine.add(
            'detect_face',
            detect_face(),
            transitions={'succeeded': 'detect_print','aborted' : 'aborted_info'})
        # it prints the standard error
        smach.StateMachine.add(
            'aborted_info',
            detect_Face_error(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})
         # it prints the standard error
        smach.StateMachine.add(
            'detect_print',
            detect_Face_print(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'detect_face_introspection', sm, '/DFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
