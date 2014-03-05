#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""


import rospy
import smach
from smach_ros import ServiceState
from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest,  StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse
from rospy.core import rospyinfo
from pal_detection_msgs.srv._StartEnrollment import StartEnrollment,\
	StartEnrollmentRequest

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

import random

class waitstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted']) #todo: i have to delate de output_key

    def execute(self, userdata):
        print "Dummy state just to change learn_face"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class learn_face(smach.StateMachine):
    """
    Executes a SM that does the proces off enrollment.
    It moves the robot to the enter door, call enter_room,
    pass the inspection (now we have a dummy state that only waits 5 secs)
    and go to the exit door. We assume that the exit door will be closed.


    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters


    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        # it's time that the robot will be learning a face
        self.userdata.time_to_learn=3000
        self.userdata.name='pepe'
        
        
        with self:
            @smach.cb_interface(input_keys=['name'])
            def face_start_request_cb(userdata, request):
                start_request = StartEnrollmentRequest()
                start_request.name=''
                return start_request
               
              
            def stop_response_cb(userdata, response):
                resulta=StopEnrollmentResponse()
                #resulta=StopEnrollmentResponse()
                #respounse=StopEnrollmentResponse()
                resulta.enrollment_ok=response.enrollment_ok
                resulta.error_msg=response.error_msg
                resulta.numFacesEnrolled=response.numFacesEnrolled
                print str(resulta.enrollment_ok)
                print str(resulta.error_msg)
                print str(resulta.numFacesEnrolled)
                
                return 'succeeded'
              
            self.userdata.name='pepe'    
               
            
            smach.StateMachine.add('wais',
                               waitstate(),
                               transitions={'succeeded':'start_enrollment','aborted' : 'aborted','preempted':'preempted'})
            
            smach.StateMachine.add('start_enrollment',
                               ServiceState('/pal_face/start_enrollment',
                                            StartEnrollment,
                                            request_cb = face_start_request_cb,
                                            input_keys = ['name']),
                               transitions={'succeeded':'wait_state','aborted' : 'aborted','preempted':'preempted'})
            # We prepare the information to go to the init door
            smach.StateMachine.add(
                                'wait_state',
                                waitstate(),
                                transitions={'succeeded': 'stop_enrollment', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
                                
            smach.StateMachine.add('stop_enrollment',
                              ServiceState('/pal_face/stop_enrollment',
                                           StopEnrollment,
                                           response_cb = stop_response_cb),
                              transitions={'succeeded':'succeeded','aborted': 'aborted','preempted':'preempted'})
      
       	
        # Go to the init door




                 




