#! /usr/bin/env python

import roslib
#roslib.load_manifest('pal_smach_utils')
import rospy
import smach
from smach import *

from util_states.global_common import *
from util_states.topic_reader import *
from std_msgs.msg import *
from speech_states.activate_asr import *
from speech_states.deactivate_asr import *

from pal_interaction_msgs.msg import *
from pal_interaction_msgs.srv import *

from speech_states.say import text_to_say
from sound_action import SpeakActionState #TODO: substitute all SpeakActionState for text_to_say
from pal_smach_utils.speech.did_you_say_yes_or_no_sm import HearingConfirmationSM
from pal_smach_utils.utils.timeout_container import SleepState

MOVE_BASE_ACTION_NAME = 'move_base'


class ProcessCommandState(smach.State):

        def __init__(self):
                smach.State.__init__(self,
                                     outcomes=[succeeded, preempted, aborted],
                                     input_keys=['in_heard'],
                                     output_keys=['value_heard_out'])

        def execute(self, userdata):
            userdata.value_heard_out = userdata.in_heard.tags[0].value
            return succeeded


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
                smach.State.__init__(self, outcomes=[succeeded, preempted, aborted], input_keys=['userSaidData'])
                self._intro_text = intro_text

        def execute(self, userdata):
                rospy.loginfo('%s: %s', self._intro_text, str(userdata.userSaidData))
                return succeeded


class RecognizeCommand(smach.State):
        def __init__(self,  command_a='jacke', command_b='angy'):
            smach.State.__init__(self, outcomes=['valid_command', 'notvalid_command', preempted, aborted], input_keys=['speechData'])
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
                smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
                with self:

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               activate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: 'HEAR_COMMAND'})

                        smach.StateMachine.add('HEAR_COMMAND',
                                               TopicReaderState(topic_name='usersaid', msg_type=asrresult, timeout=15),
                                               transitions={aborted: 'HEAR_COMMAND', succeeded: 'PROCESS_COMMAND', preempted: preempted},
                                               remapping={'message': 'userSaidData'})

                        smach.StateMachine.add('PROCESS_COMMAND',
                                               ProcessCommandState(),
                                               transitions={succeeded: 'COMMAND_CONFIRMATION' if ask_for_confirmation else 'PRINT_MESSAGE',
                                                            preempted: preempted,
                                                            aborted: aborted},
                                               remapping={'in_heard': 'userSaidData',
                                                          'value_heard_out': 'word_heard'})

                        if ask_for_confirmation:
                            smach.StateMachine.add('COMMAND_CONFIRMATION',
                                                   HearingConfirmationSM(grammar_to_reset_when_finished=GRAMMAR_NAME),
                                                   transitions={'correct_word_heard': 'PRINT_MESSAGE',
                                                                'wrong_word_heard': 'SLEEP_STATE',
                                                                preempted: preempted,
                                                                aborted: aborted},
                                                   remapping={'in_message_heard': 'word_heard'})

                            smach.StateMachine.add('SLEEP_STATE',
                                                   SleepState(0.5),
                                                   transitions={succeeded: 'HEAR_COMMAND',
                                                                preempted: preempted})

                        smach.StateMachine.add('PRINT_MESSAGE',
                                               PrintUserData(),
                                               transitions={succeeded: 'DISABLE_GRAMMAR', preempted: preempted})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               deactivate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: succeeded})


class askMissingInfo(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME='robocup/locations', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
                with self:

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               activate_ars(GRAMMAR_NAME),
                                               transitions={succeeded: 'ASK_LOCATION'})

                        def askData(userdata):
                          text = "I don't know where the " + userdata.object_name + 'is. Do you know where could I find it?'
                          return text

                        smach.StateMachine.add('ASK_LOCATION',
                                                SpeakActionState(text_cb=askData, input_keys=['object_name']),
                                                transitions={succeeded: 'HEAR_COMMAND', aborted: aborted})
                        
                        def bring_location_cb(userdata, message):
                            # actiontag = [tag for tag in message.tags if tag.key == 'action']
                            locationtag = [tag for tag in message.tags if tag.key == 'location']
                            try: 
                              userdata.location_name = locationtag[0].value
                              return succeeded
                            except:
                              return aborted

                        smach.StateMachine.add('HEAR_COMMAND',
                                               TopicReaderState(topic_name='usersaid', msg_type=asrresult, timeout=10, callback=bring_location_cb, output_keys=['location_name']),
                                               transitions={aborted: 'HEAR_COMMAND', succeeded: 'CONFIRM_OBJECT', preempted: preempted},
                                               remapping={'message': 'userSaidData'})


                        smach.StateMachine.add('PRINT_MESSAGE',
                                               PrintUserData(),
                                               transitions={succeeded: 'RECOGNIZE_COMMAND', preempted: preempted})

                        def confirm_object(userdata):
                          return "Okay! I'll go to %s." % (userdata.location_name)

                        smach.StateMachine.add('CONFIRM_OBJECT',
                                               SpeakActionState(text_cb=confirm_object, input_keys=['location_name']),
                                               transitions={succeeded: 'DISABLE_GRAMMAR',
                                                            aborted: 'DISABLE_GRAMMAR'})


                        smach.StateMachine.add('RECOGNIZE_COMMAND',
                                               RecognizeCommand(command_key, command_value),
                                               transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                            'valid_command': 'VALID_COMMAND',
                                                            preempted: preempted,
                                                            aborted: aborted},
                                               remapping={'speechData': 'userSaidData'})

                        intro_text = "Ok, understood."
                        smach.StateMachine.add('VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'DISABLE_GRAMMAR'})

                        intro_text = "I couldn't understand what you said. Can you repeat?"
                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'HEAR_COMMAND'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               deactivate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: succeeded})


class config_question(smach.State):
        
        def __init__(self):
          smach.State.__init__(self, outcomes=[succeeded, preempted, aborted], input_keys=['cat'], output_keys=['objectList'])

        def execute(self, userdata):
          from translator import get_category_list
          userdata.objectList = get_category_list(Category=userdata.cat)
          return succeeded

class askCategory(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME='categories', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
                GRAMMAR_NAME = 'robocup/' + GRAMMAR_NAME
                with self:

                        smach.StateMachine.add('CONFG_QUESTION',
                                                config_question(),
                                                transitions={succeeded: 'ENABLE_GRAMMAR'},
                                                remapping={'DataType': 'DataType', 'objectList': 'objectList'})

                        def catQuestion(userdata):
                          text = 'You asked me for a ' + userdata.cat + '. I could bring you ' + ', '.join(userdata.objectList) + '. which ' + userdata.cat + ' do you prefer?'
                          return text

                        smach.StateMachine.add('ASK_INFO',
                                               SpeakActionState(text_cb=catQuestion, input_keys=['cat', 'objectList']),
                                               transitions={succeeded: 'HEAR_COMMAND',
                                                            aborted: aborted})

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               activate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: 'ASK_INFO'})

                        def bring_order_cb(userdata, message):
                            # actiontag = [tag for tag in message.tags if tag.key == 'action']
                            objecttag = [tag for tag in message.tags if tag.key == 'object']
                            try: 
                              userdata.object_name = objecttag[0].value
                              return succeeded
                            except:
                              return aborted

                        smach.StateMachine.add('HEAR_COMMAND',
                                               TopicReaderState(topic_name='usersaid', msg_type=asrresult, timeout=10, callback=bring_order_cb, output_keys=['object_name']),
                                               transitions={aborted: 'HEAR_COMMAND', succeeded: 'CONFIRM_OBJECT', preempted: preempted},
                                               remapping={'message': 'userSaidData'})

                        smach.StateMachine.add('PRINT_MESSAGE',
                                               PrintUserData(),
                                               transitions={succeeded: 'RECOGNIZE_COMMAND', preempted: preempted})

                        def confirm_object(userdata):
                          return "Okay! You asked me for %s." % (userdata.object_name)

                        smach.StateMachine.add('CONFIRM_OBJECT',
                                               SpeakActionState(text_cb=confirm_object, input_keys=['object_name']),
                                               transitions={succeeded: 'DISABLE_GRAMMAR',
                                                            aborted: 'DISABLE_GRAMMAR'})


                        smach.StateMachine.add('RECOGNIZE_COMMAND',
                                               RecognizeCommand(command_key, command_value),
                                               transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                            'valid_command': 'VALID_COMMAND',
                                                            preempted: preempted,
                                                            aborted: aborted},
                                               remapping={'speechData': 'userSaidData'})

                        intro_text = "Ok, understood."
                        smach.StateMachine.add('VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'DISABLE_GRAMMAR'})

                        intro_text = "Sorry, I couldn't understand what you said. Can you repeat?"
                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'HEAR_COMMAND'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               deactivate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: succeeded })

class config_loc_question(smach.State):
        
        def __init__(self):
          smach.State.__init__(self, outcomes=[succeeded, preempted, aborted], input_keys=['cat'], output_keys=['locList'])

        def execute(self, userdata):
          from translator import get_loc_category_list
          userdata.locList = get_loc_category_list(Category=userdata.cat)
          return succeeded

class askCategoryLoc(smach.StateMachine):

        def __init__(self, GRAMMAR_NAME='categories', command_key='finn', command_value='xxx'):
                smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
                GRAMMAR_NAME='robocup/' + GRAMMAR_NAME.replace(' ', '_')
                with self:

                        smach.StateMachine.add('CONFG_QUESTION',
                                                config_loc_question(),
                                                transitions={succeeded: 'ENABLE_GRAMMAR'},
                                                remapping={'DataType': 'DataType', 'locList': 'locList'})

                        def catQuestion(userdata):
                          text = 'You said a ' + userdata.cat + '. I could go to ' + ', '.join(userdata.locList) + '. Which ' + userdata.cat + ' do you prefer?'
                          return text

                        smach.StateMachine.add('ASK_INFO',
                                               SpeakActionState(text_cb=catQuestion, input_keys=['cat', 'locList']),
                                               transitions={succeeded: 'HEAR_COMMAND',
                                                            aborted: aborted})

                        smach.StateMachine.add('ENABLE_GRAMMAR',
                                               activate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: 'ASK_INFO'})

                        def bring_order_cb(userdata, message):
                            # actiontag = [tag for tag in message.tags if tag.key == 'action']
                            objecttag = [tag for tag in message.tags if tag.key == 'loc']
                            try: 
                              userdata.loc_name = objecttag[0].value
                              return succeeded
                            except:
                              return aborted

                        smach.StateMachine.add('HEAR_COMMAND',
                                               TopicReaderState(topic_name='usersaid', msg_type=asrresult, timeout=10, callback=bring_order_cb, output_keys=['loc_name']),
                                               transitions={aborted: 'HEAR_COMMAND', succeeded: 'CONFIRM_OBJECT', preempted: preempted},
                                               remapping={'message': 'userSaidData'})

                        smach.StateMachine.add('PRINT_MESSAGE',
                                               PrintUserData(),
                                               transitions={succeeded: 'RECOGNIZE_COMMAND', preempted: preempted})

                        def confirm_object(userdata):
                          return "Okay! You asked me for %s." % (userdata.loc_name)

                        smach.StateMachine.add('CONFIRM_OBJECT',
                                               SpeakActionState(text_cb=confirm_object, input_keys=['loc_name']),
                                               transitions={succeeded: 'DISABLE_GRAMMAR',
                                                            aborted: 'DISABLE_GRAMMAR'})


                        smach.StateMachine.add('RECOGNIZE_COMMAND',
                                               RecognizeCommand(command_key, command_value),
                                               transitions={'notvalid_command': 'NOT_VALID_COMMAND',
                                                            'valid_command': 'VALID_COMMAND',
                                                            preempted: preempted,
                                                            aborted: aborted},
                                               remapping={'speechData': 'userSaidData'})

                        intro_text = "Ok, understood."
                        smach.StateMachine.add('VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'DISABLE_GRAMMAR'})

                        intro_text = "Sorry, I couldn't understand what you said. Can you repeat?"
                        smach.StateMachine.add('NOT_VALID_COMMAND',
                                               SpeakActionState(intro_text),
                                               transitions={succeeded: 'HEAR_COMMAND'})

                        smach.StateMachine.add('DISABLE_GRAMMAR',
                                               deactivate_asr(GRAMMAR_NAME),
                                               transitions={succeeded: succeeded })
                        



# vim: expandtab ts=4 sw=4
