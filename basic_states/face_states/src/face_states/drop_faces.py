#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 12:00:00 2013

@author: Roger Boldú
"""


from pal_detection_msgs.srv import SetDatabase, SetDatabaseRequest, SetDatabaseResponse
import rospy
from rospy.core import rospyinfo
import smach
from smach_ros import ServiceState


# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'



class drop_faces(smach.StateMachine):
    """
    Executes a SM that manage the database .
    Only do a service call whit name and purge option
    provided for input_key 


    Required parameters : 
    No parameters.

    Optional parameters: learning_time, by default is 5 seconds
    No optional parameters


    input keys: name, it's the name of the dataBaser
                purgeAll, “False” parameter means that we do not want to empty the database. Set to “True” in
                            order to remove previously stored faces.

    output keys: standard_error, whit the error, now at the moment we don't complete
    No io_keys.

    Nothing must be taken into account to use this SM.
    """
    def __init__(self, learning_time=5):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                                 input_keys=['name','purgeAll'], 
                                 output_keys=[])
        
        with self:
            # call request for SetDataBaseRequest
            @smach.cb_interface(input_keys=['name','purgeAll'])
            def face_database_request_cb(userdata, request):
                data_base_request = SetDatabaseRequest()
                data_base_request.databaseName=userdata.name
                data_base_request.purgeAll=userdata.purgeAll
                return data_base_request
    
            #call request to control the database
            smach.StateMachine.add('drop_face',
                               ServiceState('/pal_face/set_database',
                                            SetDatabase,
                                            request_cb = face_database_request_cb,
                                            input_keys = ['name','purgeAll']),
                               transitions={'succeeded':'succeeded','aborted' : 'aborted','preempted':'preempted'})
            




                 



