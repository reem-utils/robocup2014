#! /usr/bin/env python

import rospy
import smach




class Dumy_state(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.sleep(2)
        return 'succeded'
     

#Defining the state Machine of Learn Person
# the original document is hear:  https://github.com/reem-utils/robocup-code/blob/master/pal_smach_utils/src/pal_smach_utils/navigation/learn_person.py
class LearnPerson(smach.StateMachine):



    def __init__(self, learn_face=False):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'])

        with self:

            smach.StateMachine.add('Dummy_OFF_LearnPerson',
                                   Dumy_state(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted'})
