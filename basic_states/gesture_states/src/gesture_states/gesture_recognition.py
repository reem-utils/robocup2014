#! /usr/bin/env python
'''
Created on 05/04/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
import os
import sys
from gesture_states.gesture_detection_sm import gesture_detection_sm

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
    
class compare_gesture(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['gesture_detected', 'gesture_name'], output_keys=[])
        
    def execute(self, userdata):

        rospy.logerr("Gesture_name: " + userdata.gesture_name + " Gesture_detected: " + str(userdata.gesture_detected.Gesture_name.data))
        if userdata.gesture_name == userdata.gesture_detected.Gesture_name.data:
            rospy.loginfo("Gesture match!")
            return 'succeeded'
        else:
            rospy.loginfo("Gesture doesn't match")
            return 'aborted'
        
class prepareData(smach.State):
    
    def __init__(self, gesture):
        
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], 
                            input_keys=['gesture_name'], output_keys=['gesture_name'])
        self.gesture = gesture
        
    def execute(self, userdata):
           
        if not self.gesture and not userdata.gesture_name:
            rospy.logerr("Gesture isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.gesture_name = self.gesture if self.gesture else userdata.gesture_name
        rospy.loginfo(userdata.gesture_name)
        
        return 'succeeded'
          
class GestureRecognition(smach.StateMachine):
    """
    The robot search for a gesture and check if is the gesture that we are looking for. 
    In case that the gesture is correct, it return succeeded and the Pose where the person is. 
    Otherwise, it returned aborted. 
    You can input the gesture_name in init or in the userdata.gesture_name

    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    Input keys:
        gesture_name: string with the gesture name that you search
    Output keys:
        gesture_detected: Gesture.msg. It contains all the info of the gesture detected.  
    No io_keys.

    Nothing must be taken into account to use this SM.
    """ 
    def __init__(self, gesture = None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                        input_keys=['gesture_name'],
                                        output_keys=['gesture_detected'])

        with self:

            self.userdata.gesture_name = ""
             
            smach.StateMachine.add('PrepareData',
                    prepareData(gesture),
                    transitions={'succeeded':'search_gesture', 'aborted':'aborted'})
             
            # Wait for a gesture             
            smach.StateMachine.add(
                'search_gesture',
                gesture_detection_sm(),
                transitions={'succeeded': 'compare_gesture', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Compare Gesture 
            smach.StateMachine.add(
                "compare_gesture",
                compare_gesture(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})  
           
