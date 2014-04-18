#! /usr/bin/env python

import rospy
import os
import select
import sys
import rospkg


from pal_interaction_msgs.msg import ASRSrvRequest, ASRSrvResponse, ASREvent, ASRActivation, ASRLangModelMngmt, ASRLanguage, actiontag
from pal_interaction_msgs.srv import ASRService, ASRServiceRequest, ASRServiceResponse

class AsrService():
    """ASR Mock service 
        
        This mock service will simulate reem hearing someone saying MOCK_SAID. Also simulate grammar analysis of what it hearing.
        It will read tags.txt with the tags that you want to recognize, simulating the grammar. 
        We can input a sentence to simulate what the robbot had listen. 
        The file must be in the same folder than asr_srv.py
        
        exemple of using tags in pyton:
        
        def take_order_cb(userdata, message):
                self.last_drink_name = None
                print colors.BACKGROUND_GREEN, "MESSAGE: ", str(message), colors.NATIVE_COLOR

                actiontag = [tag for tag in message.tags if tag.key == 'action']
                drinktag = [tag for tag in message.tags if tag.key == 'object']  # drink
                if actiontag and actiontag[0].value == 'bring':
                    print "\n\nDRINK TAG: ", drinktag
                    userdata.out_drink_order = DrinkOrder(userdata.in_person_name, drinktag[0].value)
                    self.last_drink_name = drinktag[0].value
                    rospy.loginfo("==========>>> New drink: (%s, %s)" % (userdata.in_person_name, drinktag[0].value))
                    return succeeded
                return aborted
    
    """
    
    def __init__(self, fileName = None):
        rospy.loginfo("Initializing asrservice")
        self.fileName = fileName
        self.enabled_asr = False
        self.current_grammar = ""
        self.enabled_grammar = False
        self.current_lang = ""
        self.asrservice = rospy.Service('/asr_service', ASRService, self.asr_grammar_cb)
        rospy.loginfo("asr service initialized")
        self.usersaid_pub = rospy.Publisher('/asr_event', ASREvent)

    def isData(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
      
    def asr_grammar_cb(self, req):
        """Callback of asr service requests """
        resp = ASRServiceResponse()
        for curr_req in req.request.requests:

            if curr_req == req.request.ACTIVATION:
                if req.request.activation.action == ASRActivation.ACTIVATE:
                    self.enabled_asr = True
                    rospy.loginfo("ASR: Enabling speech recognizer")
                if req.request.activation.action == ASRActivation.DEACTIVATE:
                    self.enabled_asr = False
                    rospy.loginfo("ASR: Disabling speech recognizer")
#                 if req.request.activation.action == ASRActivation.PAUSE:
#                     self.enabled_asr = False
#                     rospy.loginfo("ASR: Pausing speech recognizer")
#                 if req.request.activation.action == ASRActivation.RESUME:
#                     self.enabled_asr = True
#                     rospy.loginfo("ASR: Resuming speech recognizer")
                    
            elif curr_req == req.request.GRAMMAR:
                if req.request.model.action == ASRLangModelMngmt.ENABLE:
                    self.enabled_grammar = True
                    self.current_grammar = req.request.model.modelName
                    rospy.loginfo("ASR: Enabling grammar '%s'" % req.request.model.modelName)
                if req.request.model.action == ASRLangModelMngmt.DISABLE:
                    self.enabled_grammar = False
                    self.current_grammar = ""
                    rospy.loginfo("ASR: Disabling grammar")
#                 if req.request.grammar.action == ASRLangModelMngmt.LOAD:
#                     self.current_grammar = req.request.grammar.grammarName
#                     rospy.loginfo("ASR: Loading grammar '%s'" % ASRLangModelMngmt.grammarName)
#                 if req.request.grammar.action == ASRLangModelMngmt.UNLOAD:
#                     self.current_grammar = ""
#                     rospy.loginfo("ASR: Unloading grammar")
                    
            elif curr_req == req.request.LANGUAGE:
                self.current_lang = req.request.lang.language
                rospy.loginfo("ASR: Changing lang to: '%s'" % req.request.lang.language)
                print "Write a sentence:"
                
            #elif curr_req == ASRServiceRequest.request.STATUS: # always return status
            
            resp.response.status.active = self.enabled_asr
            resp.response.status.enabled_grammar = self.current_grammar
            resp.response.status.language = self.current_lang
            resp.response.error_msg = ""
            resp.response.warn_msg = ""
                            
        return resp
                 
        
    def run(self):
        """Publishing usersaid when grammar is enabled """
        # TODO: add tags, add other fields, take into account loaded grammar to put other text in the recognized sentence
        while not rospy.is_shutdown():

            if self.enabled_asr and self.enabled_grammar:

                #Check for a new sentence
                if self.isData():

                    MOCK_SAID = sys.stdin.readline()
                    recognized_sentence = ASREvent()
                    recognized_sentence.recognized_utterance.text = MOCK_SAID[:len(MOCK_SAID)-1:] 
                    recognized_sentence.recognized_utterance.confidence = recognized_sentence.recognized_utterance.CONFIDENCE_MAX
                    recognized_sentence.event_id=2
                    # Tags from file
                    with open(self.fileName) as myfile:
                        for line in myfile:
                            name, var = line.partition("=")[::2]
                            tag = actiontag()
                            tag.key = name
                            tag.value = var[:len(var)-1:] 
                            recognized_sentence.recognized_utterance.tags.append(tag);
                    
                    recognized_sentence.active = self.enabled_asr
                    self.usersaid_pub.publish(recognized_sentence)

            rospy.sleep(1)


                
if __name__ == '__main__':
    rospy.init_node('asr_srv')
    rospy.loginfo("Initializing asr_srv")
    # Esta en la misma carpeta, hace falta?
    rospack_instance = rospkg.RosPack()
    asr_mock_path = rospack_instance.get_path("asr_mock")
    #pathFile = os.path.expanduser("~") + "/catkin_ws/src/robocup2014/reem_mocks/asr_mock/src/tags.txt"
    pathFile = os.path.expanduser(asr_mock_path) + "/src/tags.txt"
    if os.path.exists(pathFile): # this should check if its a file not just "exists"
        asr = AsrService(pathFile)
        asr.run()
    else:
        print "File doesn't exists. Maybe you need to change your path =D"
  

# vim: expandtab ts=4 sw=4
