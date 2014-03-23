#! /usr/bin/env python

import rospy
import smach




class Dumy_state(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'],input_keys=['standard_error'],output_keys=['standard_error'])

    def execute(self, userdata):
        rospy.sleep(2)
        userdata.standard_error="Dummy"
        return 'succeeded'
     

#Defining the state Machine of Learn Person
class follow_me_3rd(smach.StateMachine):



    def __init__(self, learn_face=False):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],output_keys=['standard_error'])
        
        with self:
            self.userdata.standar_error="ok"
            smach.StateMachine.add('Dummy_OFF_follow_me_3rd',
                                   Dumy_state(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted','preempted':'preempted'})
