#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLLOW_ME.PY ###
import smach
import rospy
from speech_states.say import text_to_say
"""
@author: Roger Boldu
"""
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from speech_states.say import text_to_say
from speech_states.listen_and_check_word import ListenWordSM
from pipol_tracker_pkg.msg import personArray,person
from util_states.topic_reader import topic_reader
from face_states.new_database_and_learn import new_database_and_learn
from std_msgs.msg import Int32

# It's only becouse i can't import the file... i can't understand
class fake_id(smach.State):

    def __init__(self): 
        smach.State.__init__(self, input_keys=[],
                             output_keys=[],
                             outcomes=['succeeded','aborted', 'preempted'])

    
        self.follow_pub = rospy.Publisher('/follow_me/id', Int32, latch=True)    
    def execute(self, userdata):
        self.follow_pub.publish(0)
        return 'succeeded'

