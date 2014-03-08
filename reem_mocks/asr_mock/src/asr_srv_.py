#! /usr/bin/env python

import rospy

#from pal_interaction_msgs.msg import asrupdate, asrresult
#from pal_interaction_msgs.srv import recognizerService
#from pal_interaction_msgs.srv import recognizerServiceRequest, recognizerServiceResponse

from pal_interaction_msgs.msg import ASRSrvRequest, ASRSrvResponse, ASREvent, ASRActivation, ASRGrammarMngmt, ASRLanguage
from pal_interaction_msgs.srv import ASRService, ASRServiceRequest, ASRServiceResponse

class AsrService():
    """ASR Mock service """
    
    def __init__(self):
        rospy.loginfo("Initializing asrservice")
        self.enabled_asr = False
        self.current_grammar = ""
        self.enabled_grammar = False
        self.current_lang = ""
        self.asrservice = rospy.Service('/asr_server', ASRService, self.asr_grammar_cb)
        rospy.loginfo("asr service initialized")
        self.usersaid_pub = rospy.Publisher('/asr_event', ASREvent)
        
    def asr_grammar_cb(self, req):
        """Callback of asr service requests """
        #req = ASRServiceRequest() # trick to autocomplete
        resp = ASRServiceResponse()
        for curr_req in req.request.requests:
            rospy.loginfo("In the for loop")
            if curr_req == req.request.ACTIVATION:
                if req.request.activation.action == ASRActivation.ACTIVATE:
                    self.enabled_asr = True
                    rospy.loginfo("ASR: Enabling speech recognizer")
                if req.request.activation.action == ASRActivation.DEACTIVATE:
                    self.enabled_asr = False
                    rospy.loginfo("ASR: Disabling speech recognizer")
                if req.request.activation.action == ASRActivation.PAUSE:
                    self.enabled_asr = False
                    rospy.loginfo("ASR: Pausing speech recognizer")
                if req.request.activation.action == ASRActivation.RESUME:
                    self.enabled_asr = True
                    rospy.loginfo("ASR: Resuming speech recognizer")
                    
            elif curr_req == req.request.GRAMMAR:
                if req.request.grammar.action == ASRGrammarMngmt.ENABLE:
                    self.enabled_grammar = True
                    rospy.loginfo("ASR: Enabling grammar")
                if req.request.grammar.action == ASRGrammarMngmt.DISABLE:
                    self.enabled_grammar = False
                    rospy.loginfo("ASR: Disabling grammar")
                if req.request.grammar.action == ASRGrammarMngmt.LOAD:
                    self.current_grammar = req.request.grammar.grammarName
                    rospy.loginfo("ASR: Loading grammar '%s'" % ASRGrammarMngmt.grammarName)
                if req.request.grammar.action == ASRGrammarMngmt.UNLOAD:
                    self.current_grammar = ""
                    #PREGUNTAR COM TRACTAR DIFERENTS GRAMARS EN PARALEL
                    rospy.loginfo("ASR: Unloading grammar")
                    
            elif curr_req == req.request.LANGUAGE:
                self.current_lang = req.request.lang.language
                rospy.loginfo("ASR: Changing lang to: '%s'" % req.request.lang.language)
                
            #elif curr_req == ASRServiceRequest.request.STATUS: # always return status
            
            resp.response.status.active = self.enabled_asr
            resp.response.status.enabled_grammar = self.current_grammar
            resp.response.status.langauge = self.current_lang
            resp.response.error_msg = ""
            resp.response.warn_msg = ""
                            
        return resp
                 
        
    
        
    def run(self):
        """Publishing usersaid when grammar is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():
            if self.enabled_asr:
                recognized_sentence = ASREvent()
                recognized_sentence.recognized_utterance.text = "Kill Roger"
                recognized_sentence.recognized_utterance.confidence = ASREvent.recognized_utterance.CONFIDENCE_MAX
                self.usersaid_pub.publish(recognized_sentence)
            rospy.sleep(1)
        
        
                
if __name__ == '__main__':
    rospy.init_node('asr_srv')
    rospy.loginfo("Initializing asr_srv")
    asr = AsrService()
    asr.run()
  

# vim: expandtab ts=4 sw=4
