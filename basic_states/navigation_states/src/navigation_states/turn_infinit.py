#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
from smach_ros import ServiceState
from navigation_states.srv import *

import math

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

degree=45

'''
@ this state machine use check_elevator, it turns the enable on
@ it will return succeeded when it's in the elevator
you can pass an optional param that indicates when starts to loock
'''

class turn_infinit(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, direction='left'):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted','aborted'],
            input_keys=[])

        self.direction=direction
        

        with self:
            

            def nav_turn(userdata, request):
                turn_request=NavigationTurnRequest()
                
                if self.direction=='left':
                    self.angle=-45
                else :
                    self.angle=45
                    
                turn_request.degree=self.angle
                turn_request.enable=True
                return turn_request
            
            
            smach.StateMachine.add('turn',ServiceState('/turn',NavigationTurn,request_cb = nav_turn),
                            transitions={'succeeded':'turn','aborted' : 'turn','preempted':'preempted'})      
            
                                            

