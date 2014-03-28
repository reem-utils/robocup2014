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
class follow_me_3rd(smach.StateMachine):



    def __init__(self, learn_face=False):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'])

        with self:

            smach.StateMachine.add('Dummy_OFF_follow_me_3rd',
                                   Dumy_state(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted'})
