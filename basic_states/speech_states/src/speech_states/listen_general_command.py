#! /usr/bin/env python

import roslib
#roslib.load_manifest('pal_smach_utils')
import rospy
import smach
from smach import *

from util_states.global_common import *
from std_msgs.msg import *
from speech_states.activate_asr import ActivateASR
from speech_states.read_asr import ReadASR
from speech_states.deactivate_asr import DeactivateASR

from pal_interaction_msgs.msg import *
from pal_interaction_msgs.srv import *

from speech_states.say import text_to_say
#from pal_smach_utils.speech.did_you_say_yes_or_no_sm import HearingConfirmationSM
from speech_states.say_yes_or_no import SayYesOrNoSM
#from pal_smach_utils.utils.timeout_container import SleepState

from pal_interaction_msgs.msg._ASREvent import ASREvent

MOVE_BASE_ACTION_NAME = 'move_base'


class ProcessCommandState(smach.State):

        def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['in_heard'],
                                     output_keys=['value_heard_out'])

        def execute(self, userdata):
            userdata.value_heard_out = userdata.in_heard.tags[0].value
            return 'succeeded'


class SleepState(smach.State):

        def __init__(self,time):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'preempted', 'aborted'])

        def execute(self, userdata):
            rospy.sleep(self.time)
            return 'succeeded'


class PrintUserData(smach.State):

        '''
        This State Prints whatever you want.
        input_keys =    'userSaidData' , is the data that you want to print
        intro text =    is what you want as intro before the data.
                        Default message is '@@@@ This is the Message '
        use --> smach.StateMachine.add( 'PRINT_MESSAGE',
                                         PrintUserData('what you want as intro'),
                                         transitions = { succeeded:succeeded,
                                                         preempted:preempted},
                                         remapping = {'userSaidData':'nave_state_read'}
        '''

        def __init__(self, intro_text='@@@@ This is the Message '):
                smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['userSaidData'])
                self._intro_text = intro_text

        def execute(self, userdata):
                rospy.loginfo('%s: %s', self._intro_text, str(userdata.userSaidData))
                return 'succeeded'


class RecognizeCommand(smach.State):
        def __init__(self,  command_a='jacke', command_b='angy'):
            smach.State.__init__(self, outcomes=['valid_command', 'notvalid_command', 'preempted', 'aborted'], input_keys=['speechData'])
            self._command_a = command_a
            self._command_b = command_b

        def execute(self, userdata):

                # TODO: add confidence check before returning valid recognition (confirmation dialogue)
                goto_tags = [tag for tag in userdata.speechData.tags if tag.key == self._command_a]
            #   print "\n\n\n gototags es: "
            #   print goto_tags
                if goto_tags:
                    if self._command_b == goto_tags[0].value:
                        return 'valid_command'
                return 'notvalid_command'


class RecogCommand(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME, command_key='finn', command_value='xxx', ask_for_confirmation=False):
                smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])
                with self:

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               ActivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'HEAR_COMMAND'})
 
                        smach.StateMachine.add('HEAR_COMMAND',
                                               ReadASR(),
                                               transitions={'aborted': 'HEAR_COMMAND', 'succeeded': 'PROCESS_COMMAND', 'preempted': 'preempted'},
                                               remapping={'asr_userSaid': 'userSaidData', 'asr_userSaid_tags':'userSaidTags'})
                        
                        smach.StateMachine.add('PROCESS_COMMAND',
                                               ProcessCommandState(),
                                               transitions={'succeeded': 'ASK_COMMAND_CONFIRMATION' if ask_for_confirmation else 'PRINT_MESSAGE',
                                                            'preempted': 'preempted',
                                                            'aborted': 'aborted'},
                                               remapping={'in_heard': 'userSaidData',
                                                          'value_heard_out': 'word_heard'})

                        if ask_for_confirmation:
                            smach.StateMachine.add('ASK_COMMAND_CONFIRMATION',
                                                   text_to_say('Did you say ' + userdata.word_heard + '?'),
                                                   transitions={'succeeded': 'LISTEN_COMMAND_CONFIRMATION',
                                                                'aborted': 'SLEEP_STATE',
                                                                'preempted': 'preempted'})
                            
                            smach.StateMachine.add('LISTEN_COMMAND_CONFIRMATION',
                                                   SayYesOrNoSM(),
                                                   transitions={'succeeded': 'DISABLE_GRAMMAR',
                                                                'aborted': 'SLEEP_STATE',
                                                                'preempted': 'preempted'},
                                                   remapping={'in_message_heard': 'word_heard'})

                            smach.StateMachine.add('SLEEP_STATE',
                                                   SleepState(0.5),
                                                   transitions={'succeeded': 'HEAR_COMMAND',
                                                                'preempted': 'preempted'})

                        smach.StateMachine.add('PRINT_MESSAGE',
                                               PrintUserData(),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR', 'preempted': 'preempted'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               DeactivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'succeeded'})


class BringLocationAsk(smach.State):

        def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['userSaidTags'],
                                     output_keys=['location_name']) 
                
        def execute(self, userdata):
            try: 
                  for tag in userdata.userSaidTags :
                    if tag.key == 'location':
                      userdata.location_name = tag.value
                  return 'succeeded'
            except:
                  print 'faaaail object'
                  return 'aborted'

class BringOrderObject(smach.State):

        def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['userSaidTags'],
                                     output_keys=['object_name'])

        def execute(self, userdata):
            try: 
                  for tag in userdata.userSaidTags :
                    if tag.key == 'object':
                      userdata.object_name = tag.value
                  return 'succeeded'
            except:
                  print 'faaaail object'
                  return 'aborted'
              
class BringOrderLoc(smach.State):

        def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['userSaidTags'],
                                     output_keys=['loc_name'])

        def execute(self, userdata):
                  # actiontag = [tag for tag in message.tags if tag.key == 'action']
#                 objecttag = [tag for tag in self.userdata.userSaidTags if tag.key == 'loc']
#                 try: 
#                   self.userdata.loc_name = objecttag[0].value
            try: 
                  for tag in userdata.userSaidTags :
                    if tag.key == 'loc':
                      userdata.loc_name = tag.value
                  return 'succeeded'
            except:
                  print 'faaaail location'
                  return 'aborted'
              
class askMissingInfo(smach.StateMachine):

        def __init__(self, Type, objectName, GRAMMAR_NAME='robocup/locations', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])
                
                self.userdata.dataType = Type
                self.userdata.object_name = objectName
                self.userdata.location_name = ''
                self.userdata.grammar_name = GRAMMAR_NAME
                self.userdata.tts_wait_before_speaking = 0
                self.userdata.tts_text = ''
                self.userdata.tts_lang = ''
                
                with self:

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               ActivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'ASK_LOCATION'})
                        
                        smach.StateMachine.add('ASK_LOCATION',
                                                text_to_say("I don't know where the " + self.userdata.object_name + ' is. Do you know where could I find it?'),
                                                transitions={'succeeded': 'HEAR_COMMAND', 'aborted': 'aborted'})

                        smach.StateMachine.add('HEAR_COMMAND',
                                               ReadASR(),
                                               transitions={'aborted': 'HEAR_COMMAND', 'succeeded': 'BRING_LOCATION', 'preempted': 'preempted'},
                                               remapping={'asr_userSaid': 'userSaidData', 'asr_userSaid_tags':'userSaidTags'})
                        
                        smach.StateMachine.add('BRING_LOCATION',
                                               BringLocationAsk(),
                                               transitions={'aborted': 'HEAR_COMMAND', 'succeeded': 'CONFIRM_OBJECT', 'preempted': 'preempted'},
                                               remapping={'location_name': 'location_name'})

#                         smach.StateMachine.add('PRINT_MESSAGE',
#                                                PrintUserData(),
#                                                transitions={'succeeded': 'RECOGNIZE_COMMAND', 'preempted': 'preempted'})                         
                      
                        smach.StateMachine.add('CONFIRM_OBJECT',
                                                text_to_say("Okay! I'll go to"+ self.userdata.location_name),
                                                transitions={'succeeded': 'DISABLE_GRAMMAR', 'aborted': 'DISABLE_GRAMMAR'})
                         
                        smach.StateMachine.add('RECOGNIZE_COMMAND', RecognizeCommand(command_key, command_value),
                                                transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                'valid_command': 'VALID_COMMAND', 'preempted': 'preempted', 'aborted': 'aborted'},
                                                remapping={'speechData': 'userSaidData'})
 
                        smach.StateMachine.add('VALID_COMMAND',
                                               text_to_say("Ok, understood."),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR'})
 
                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               text_to_say("I couldn't understand what you said. Can you repeat?"),
                                               transitions={'succeeded': 'HEAR_COMMAND'})
 
                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               DeactivateASR(),
                                               transitions={'succeeded': 'succeeded'})

            
class config_question(smach.State):
         
        def __init__(self):
          smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['cat'], output_keys=['objectList'])
 
        def execute(self, userdata):
          from translator import get_category_list
          userdata.objectList = get_category_list(Category=userdata.cat)
          
          return 'succeeded'

class askCategory(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME='categories', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'])
                #GRAMMAR_NAME = 'robocup/' + GRAMMAR_NAME
                
                self.userdata.cat = GRAMMAR_NAME
                self.userdata.objectList = []
                self.userdata.location_name = ''
                self.userdata.grammar_name = GRAMMAR_NAME
                self.userdata.tts_wait_before_speaking = 0
                self.userdata.tts_text = ''
                self.userdata.tts_lang = ''
                
                with self:

                        smach.StateMachine.add('CONFG_QUESTION',
                                                config_question(),
                                                transitions={'succeeded': 'ENABLE_GRAMMAR'},
                                                remapping={'cat': 'cat', 'objectList': 'objectList'})

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               ActivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'ASK_INFO'})
                        
                        smach.StateMachine.add('ASK_INFO',
                                               text_to_say('You asked me for a ' + self.userdata.cat + '. I could bring you ' + ', '.join(self.userdata.objectList) + '. which ' + self.userdata.cat + ' do you prefer?'),
                                               transitions={'succeeded': 'HEAR_COMMAND_OBJECT',
                                                            'aborted': 'aborted'})
                     
                        smach.StateMachine.add('HEAR_COMMAND_OBJECT',
                                               ReadASR(),
                                               transitions={'aborted': 'HEAR_COMMAND_OBJECT', 'succeeded': 'BRING_ORDER', 'preempted': 'preempted'},
                                               remapping={'asr_userSaid': 'userSaidData', 'asr_userSaid_tags':'userSaidTags'})

                        smach.StateMachine.add('BRING_ORDER',
                                               BringOrderObject(),
                                               transitions={'aborted': 'HEAR_COMMAND_OBJECT', 'succeeded': 'CONFIRM_OBJECT', 'preempted': 'preempted'})
                                               
#                         smach.StateMachine.add('PRINT_MESSAGE',
#                                                PrintUserData(),
#                                                transitions={'succeeded': 'RECOGNIZE_COMMAND', 'preempted': 'preempted'})
                        
                        smach.StateMachine.add('CONFIRM_OBJECT', text_to_say("Okay! I'll go to " + self.userdata.location_name),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR', 'aborted': 'DISABLE_GRAMMAR'})


                        smach.StateMachine.add('RECOGNIZE_COMMAND', 
                                               RecognizeCommand(command_key, command_value),
                                               transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                            'valid_command': 'VALID_COMMAND',
                                                            'preempted': 'preempted',
                                                            'aborted': 'aborted'},
                                               remapping={'speechData': 'userSaidData'})

                        smach.StateMachine.add('VALID_COMMAND',
                                               text_to_say("Ok, understood."),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR'})

                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               text_to_say("Sorry, I couldn't understand what you said. Can you repeat?"),
                                               transitions={'succeeded': 'HEAR_COMMAND_OBJECT'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               DeactivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'succeeded' })

class config_loc_question(smach.State):
        
        def __init__(self):
          smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['cat'], output_keys=['locList'])

        def execute(self, userdata):
          from translator import get_loc_category_list
          userdata.locList = get_loc_category_list(Category=userdata.cat)
          return 'succeeded'

class askCategoryLoc(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME='categories', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])
                #GRAMMAR_NAME='robocup/' + GRAMMAR_NAME.replace(' ', '_')
                
                self.userdata.cat = GRAMMAR_NAME
                self.userdata.objectList = []
                self.userdata.locList = []
                self.userdata.loc_name = ''
                self.userdata.grammar_name = GRAMMAR_NAME
                self.userdata.tts_wait_before_speaking = 0
                self.userdata.tts_text = ''
                self.userdata.tts_lang = ''
                
                with self:

                        smach.StateMachine.add('CONFG_QUESTION',
                                                config_loc_question(),
                                                transitions={'succeeded': 'ENABLE_GRAMMAR'},
                                                remapping={'DataType': 'DataType', 'locList': 'locList'})

                       # def catQuestion(userdata):
                        #  text = 'You said a ' + userdata.cat + '. I could go to ' + ', '.join(userdata.locList) + '. Which ' + userdata.cat + ' do you prefer?'
                         # return text

                        #blabla
                        smach.StateMachine.add('ASK_INFO',
                                               text_to_say('You said a ' + self.userdata.cat + '. I could go to ' + ', '.join(self.userdata.locList) + '. Which ' + self.userdata.cat + ' do you prefer?'),
                                               transitions={'succeeded': 'HEAR_COMMAND',
                                                            'aborted': 'aborted'})

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               ActivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'ASK_INFO'})


                        smach.StateMachine.add('HEAR_COMMAND',
                                               ReadASR(),
                                               transitions={'aborted': 'HEAR_COMMAND', 'succeeded': 'BRING_ORDER', 'preempted': 'preempted'},
                                               remapping={'asr_userSaid': 'userSaidData', 'asr_userSaid_tags':'userSaidTags'})

                        smach.StateMachine.add('BRING_ORDER',
                                               BringOrderLoc(),
                                               transitions={'aborted': 'HEAR_COMMAND', 'succeeded': 'CONFIRM_OBJECT', 'preempted': 'preempted'})
                                               
#                         smach.StateMachine.add('PRINT_MESSAGE',
#                                                PrintUserData(),
#                                                transitions={'succeeded': 'RECOGNIZE_COMMAND', 'preempted': 'preempted'})
                         
                        smach.StateMachine.add('CONFIRM_OBJECT',
                                               text_to_say("Okay! I'll go to " + self.userdata.loc_name),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR',
                                                            'aborted': 'DISABLE_GRAMMAR'})


                        smach.StateMachine.add('RECOGNIZE_COMMAND',
                                               RecognizeCommand(command_key, command_value),
                                               transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                            'valid_command': 'VALID_COMMAND',
                                                            'preempted': 'preempted',
                                                            'aborted': 'aborted'},
                                               remapping={'speechData': 'userSaidData'})

                        smach.StateMachine.add('VALID_COMMAND',
                                               text_to_say("Ok, understood."),
                                               transitions={'succeeded': 'DISABLE_GRAMMAR'})

                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               text_to_say("Sorry, I couldn't understand what you said. Can you repeat?"),
                                               transitions={'succeeded': 'HEAR_COMMAND'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               DeactivateASR(GRAMMAR_NAME),
                                               transitions={'succeeded': 'succeeded' })
                        



# vim: expandtab ts=4 sw=4
