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



        

class calculate_distance(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                             input_keys=[],output_keys=[])
    def execute(self,userdata):
        self.angle=45
        if self.distance>0 :
            self.angle=45
        else : 
            self.angle = -45
            
        self.hipo=self.distance/(math.sin(45))
        self.adyacente=self.hipo*math.cos(45)
        self.hipo=self.hipo
        return 'succeeded'
            



class nav_lateral(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, distance=1.0):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted','aborted'],
            input_keys=[])
        self.distance=distance
        

        with self:
            
            self.userdata.distance=distance
            self.userdata.angle=45.0
            self.userdata.adyacente=1.0
            
            
            self.angle=45
            if self.distance>0 :
                self.angle=45
            else : 
                self.angle = -45
            
            self.hipo=self.distance/(math.sin(45))
            self.adyacente=self.hipo*math.cos(45)
            self.hipo=self.hipo
        
            def nav_turn_start(userdata, request):
                turn_request=NavigationTurnRequest()
                turn_request.degree=-self.angle
                turn_request.enable=True
                return turn_request
            
            def nav_turn_finsish(userdata, request):
                turn_request=NavigationTurnRequest()
                turn_request.degree=self.angle
                turn_request.enable=True
                return turn_request
            def nav_back(userdata, request):
                reverse_request=NavigationGoBackRequest()
                reverse_request.meters=self.hipo
                reverse_request.enable=True
                return reverse_request
            def nav_forward(userdata, request):
                forward_request=NavigationGoForwardRequest()
                forward_request.meters=self.adyacente
                forward_request.enable=True
                return forward_request
                            
           # smach.StateMachine.add('INIT_VAR',
            #                   calculate_distance(),
             #                  transitions={'succeeded': "First_turn",'aborted':'aborted', 'preempted':'preempted'})
            
            smach.StateMachine.add('First_turn',ServiceState('/turn',NavigationTurn,request_cb = nav_turn_start),
                            transitions={'succeeded':'navigation_back','aborted' : 'First_turn','preempted':'preempted'})      
            
            smach.StateMachine.add('navigation_back',
                            ServiceState('/reverse',
                            NavigationGoBack,
                            request_cb = nav_back),
                            transitions={'succeeded':'Second_turn','aborted' : 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add('Second_turn',
                            ServiceState('/turn',
                            NavigationTurn,
                            request_cb = nav_turn_finsish),
                            transitions={'succeeded':'navigation_forward','aborted' : 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add('navigation_forward',
                            ServiceState('/forward',
                            NavigationGoForward,
                            request_cb = nav_forward),
                            transitions={'succeeded':'succeeded','aborted' : 'aborted','preempted':'preempted'})
                                            

