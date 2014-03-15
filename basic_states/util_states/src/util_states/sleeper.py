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

    def __init__(self):
        
        smach.State.__init__(self, input_keys=['sleep_time'], output_keys=[], outcomes=['succeeded', 'aborted'])       
        self.time_init = rospy.Time.now()

    def execute(self, userdata):  
       
        rospy.sleep(userdata.sleep_time)
        return 'succeeded'
        