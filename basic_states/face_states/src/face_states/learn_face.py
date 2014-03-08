#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""


from pal_detection_msgs.srv import StartEnrollmentResponse, StartEnrollmentRequest,  StartEnrollment
from pal_detection_msgs.srv import StopEnrollment, StopEnrollmentRequest, StopEnrollmentResponse
import rospy
from rospy.core import rospyinfo
import smach
from smach_ros import ServiceState


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


class waitstate(smach.State):
    def __init__(self, learning_time):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted']) #todo: i have to delate de output_key
        self.learning_time = learning_time
    def execute(self, userdata):
        rospy.loginfo("Learning Face")
        rospy.sleep(self.learning_time)
        return 'succeeded'


class learn_face(smach.StateMachine):
    """
    Executes a SM that does the process off enrollment.
    It call a enrollmentStard Service,
    it waits learning_time,
    and then stops the enrollment, 


    Required parameters : 
    No parameters.

    Optional parameters: learning_time, by default is 5 seconds
    No optional parameters


    input keys: name, it's the name of the person who will enroll
    output keys: standard_error, whit the error
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self, learning_time=5):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name'], 
                                 output_keys=['standard_error'])
        
        with self:
            # call request for StardEnrollment
            @smach.cb_interface(input_keys=['name'])
            def face_start_request_cb(userdata, request):
                start_request = StartEnrollmentRequest()
                start_request.name=userdata.name
                return start_request
            # call response for stop
            # it returns the enrollment, true o false, if it found   
            @smach.cb_interface(output_keys=['standard_error'])
            def stop_response_cb(userdata, response):
                resultat=StopEnrollmentResponse() 
                resultat.enrollment_ok=response.enrollment_ok
                resultat.error_msg=response.error_msg
                resultat.numFacesEnrolled=response.numFacesEnrolled
                
                if resultat.enrollment_ok == False:
                    rospy.loginfo(FAIL+"Learning Face Error  " +
                                str(resultat.enrollment_ok)+"  "+str(resultat.error_msg)
                                +"  "+str(resultat.numFacesEnrolled)+ENDC)
                    self.userdata.standard_error='face not found'
                    return 'aborted'
                else:
                    self.userdata.standard_error='ok, face found'
                    return 'succeeded'
                   
            #call request of start enrollment
            smach.StateMachine.add('start_enrollment',
                               ServiceState('/pal_face/start_enrollment',
                                            StartEnrollment,
                                            request_cb = face_start_request_cb,
                                            input_keys = ['name']),
                               transitions={'succeeded':'wait_state','aborted' : 'aborted','preempted':'preempted'})
            # Wait learning_time, that the robot will be learning the face
            smach.StateMachine.add(
                                'wait_state',
                                waitstate(learning_time),
                                transitions={'succeeded': 'stop_enrollment', 'aborted': 'aborted', 
                                'preempted': 'preempted'})
            
            # it returns StopEnrollmentResponse, if face not found aborted                     
            smach.StateMachine.add('stop_enrollment',
                              ServiceState('/pal_face/stop_enrollment',
                                           StopEnrollment,
                                           response_cb = stop_response_cb,
                                           output_keys = ['standard_error']),
                              transitions={'succeeded':'succeeded','aborted': 'aborted','preempted':'preempted'})
      
           
        # Go to the init door




                 



