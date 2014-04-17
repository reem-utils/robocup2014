#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

15/04/2014

"""

import rospy
import smach

class prepareData(smach.State):
    
    def __init__(self, object_name):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['object_name'], output_keys=['object_name'])
        self.object_name = object_name
        
    def execute(self, userdata):
           
        if not self.object_name and not userdata.object_name:
            rospy.logerr("Object_name isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.object_name = self.object_name if self.object_name else userdata.object_name   
        
        return 'succeeded'               

# In this state we will obtain all the information from a object
class obtain_info(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
         input_keys=['object_name', 'object_location'], 
         output_keys=['object_location','standard_error'])

    def execute(self, userdata):
      
        objectName=userdata.object_name
        
        objList = rospy.get_param("mmap/object/information")
        foundObject= False
        
        for key,value in objList.iteritems():
            if value[1] == objectName:
                object_class = value[2]
                foundObject = True  
                break    
        
        if foundObject:
            prob = 0.0
            
            pois = rospy.get_param("/mmap/object/" + object_class)
            from object_grasping_states.recognize_object import recognize_object

            for key, value in pois.iteritems():
         
                if prob < value[2]:
                    userdata.object_location = value[1]
                    prob = value[2]
                    
            if prob == 0.0:
                userdata.standard_error= "Can't find a location"
                rospy.logerr("Can't find a location for " + objectName)
            else:
                userdata.standard_error='OK'
                rospy.loginfo("Location: " + userdata.object_location)
                return 'succeeded'
        else :
            userdata.standard_error='Object not found'
            rospy.logerr(objectName + ' NOT FOUND')
            return 'aborted'


class GetObjectInfoSM(smach.StateMachine):
    """
    Executes a SM that search for object. 
    Given the object name, it search which place is the most probably that we can find it. 

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input keys:
        object_name: string with the object's name
    Output keys:
        object_location: string with the place most probably  
        standard_error: String that show what kind of error could be happened
    No io_keys.

    Nothing must be taken into account to use this SM.
    """

    def __init__(self, object_name = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                    input_keys=['object_name'],
                    output_keys=['standard_error', 'object_location'])

        with self:        
        
            smach.StateMachine.add('PrepareData',
               prepareData(object_name),
               transitions={'succeeded':'get_object_info', 'aborted':'aborted'})
            
            # Obtain all the information
            smach.StateMachine.add('get_object_info',
                   obtain_info(),
                   transitions={'succeeded': 'succeeded',
                                'aborted': 'aborted',
                                'preempted': 'preempted'})

