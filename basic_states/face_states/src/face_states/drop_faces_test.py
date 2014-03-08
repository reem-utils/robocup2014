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
from face_states.drop_faces import drop_faces
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class prepare_drop_faces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=['name','purgeAll'])

    def execute(self, userdata):
        rospy.loginfo("preparing drop face")
        userdata.name='pepe'# is the name of the database
        userdata.purgeAll=False # it means if you want to delete the database
        return 'succeeded'
    # this state is a waiting state, you can delete
class dummy_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("dummy state")
        rospy.sleep(5);
        return 'succeeded'  
    # it print the standard_error
class drop_faces_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'],input_keys=['standard_error'], output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.loginfo('info of aborted drop Face')
        rospy.loginfo( FAIL +"standard_error :==   "+str(userdata.standard_error) + ENDC)
        return 'aborted'
          
def main():
    rospy.init_node('drop_face_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        # it prepare the name and the function for drope
        smach.StateMachine.add(
            'prepare_learn_face',
            prepare_drop_faces(),
            transitions={'succeeded':'learn_face','aborted' : 'aborted','preempted':'preempted'})
        # it call the drop_face state
        smach.StateMachine.add(
            'learn_face',
            drop_faces(),
            transitions={'succeeded': 'dummy','aborted' : 'aborted_info'})
        # it prints the standard error
        smach.StateMachine.add(
            'aborted_info',
            drop_faces_error(),
            transitions={'succeeded': 'dummy', 'aborted':'dummy'})
        #dummy_state
        smach.StateMachine.add(
            'dummy',
            dummy_state(),
            transitions={'succeeded': 'succeeded', 'aborted':'aborted'})
       

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'drop_faces_introspection', sm, '/DFI_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
