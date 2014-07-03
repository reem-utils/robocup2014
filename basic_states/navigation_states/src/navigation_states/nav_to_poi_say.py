#!/usr/bin/env python

"""
@author:  Roger Boldu


19 Feb 2014
"""


#import rospy
import rospy
import smach
from nav_to_poi import nav_to_poi
from speech_states.say import text_to_say


class prepareData(smach.State):
    
    def __init__(self, tts,poi_name):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_poi_name'], output_keys=['nav_to_poi_name'])
        self.tts = tts
        
    def execute(self, userdata):
        if self.tts != None :
            userdata.tts_text=self.tts   
        
        if not self.poi_name and not userdata.nav_to_poi_name:
            rospy.logerr("Poi_name isn't set")
            return 'aborted'
        #Priority in init
        userdata.nav_to_poi_name = self.poi_name if self.poi_name else userdata.nav_to_poi_name   
        
        return 'succeeded'
    
    
class nav_to_poi_say(smach.StateMachine):
    """
    it talks and move at the same time
    """

    
    def __init__(self,tts=None,poi_name=None):
        """
        Constructor for nav_to_coord.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['nav_to_coord_goal','tts_text', 'tts_wait_before_speaking', 'tts_lang','nav_to_poi_name'],
                                 output_keys=[])
        self.tts=tts
        self.poi_name=poi_name
        
        with self: 

            smach.StateMachine.add('say',
                                   text_to_say(self.tts,wait=False),
                                   transitions={'succeeded':'nav_to_poi', 'aborted':'aborted'})

            
            smach.StateMachine.add(
                'nav_to_poi',
                nav_to_poi(self.poi_name),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'})

