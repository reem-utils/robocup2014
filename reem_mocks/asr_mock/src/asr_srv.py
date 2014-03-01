#! /usr/bin/env python

import rospy
import actionlib

from pal_interaction_msgs.msg import asrupdate, asrresult
from pal_interaction_msgs.srv import recognizerService
from pal_interaction_msgs.srv import recognizerServiceRequest, recognizerServiceResponse


class AsrService():
    """ASR Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing asrservice")
        self.current_grammar = ""
        self.enabled_grammar = False
        self.asrservice = rospy.Service('/asrservice', recognizerService, self.asr_grammar_cb)
        rospy.loginfo("asr service initialized")
        self.usersaid_pub = rospy.Publisher('/usersaid', asrresult)
        
    def asr_grammar_cb(self, req):
        """Callback of asr service requests """
        if req.asrupdate.enable_grammar:
            rospy.loginfo("ASR: Enabling grammar '%s'" % req.asrupdate.enable_grammar)
            self.current_grammar = req.asrupdate.enable_grammar
            self.enabled_grammar = True
        elif req.disable_grammar:
            rospy.loginfo("ASR: Disabling grammar '%s'" % req.asrupdate.disable_grammar)
            self.current_grammar = ""
            self.enabled_grammar = False        
        return recognizerServiceResponse()
    
        
    def run(self):
        """Publishing usersaid when grammar is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled_grammar:
                recognized_sentence = asrresult()
                recognized_sentence.text = "whatever"
                self.usersaid_pub.publish(recognized_sentence)
            rospy.sleep(1)
        
        
                
if __name__ == '__main__':
  rospy.init_node('asr_srv')
  rospy.loginfo("Initializing asr_srv")
  asr = AsrService()
  asr.run()
  

# vim: expandtab ts=4 sw=4
