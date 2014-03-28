#! /usr/bin/env python
'''
Created on 15/03/2014

@author: Cristina De Saint Germain

'''

import rospy
import smach

class Sleeper(smach.State):
    """Sleeper State

    Waits the specified time
    
    Input Keys:
        sleeper_time: Time we want to sleeper
    Output Keys:
        standard_error: String thats shows what fails
    
   """

    def __init__(self, sleep_time = None):
        smach.State.__init__(self, input_keys=['sleep_time'], output_keys=['sleep_time'], outcomes=['succeeded', 'aborted'])       
        self.sleep_time = sleep_time
        
    def execute(self, userdata):  
        
        if not self.sleep_time and not userdata.sleep_time:
            rospy.logerr("Time isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.sleep_time = self.sleep_time if self.sleep_time else userdata.sleep_time  
        
        rospy.sleep(userdata.sleep_time)
        return 'succeeded'
        