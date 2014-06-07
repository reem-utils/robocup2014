import roslib
roslib.load_manifest('gpsr')
import smach
from smach_ros import SimpleActionState
import rospy

from util_states.global_common import transform_pose#,succeeded, preempted, aborted
from speech_states.listen_to import ListenToSM  # instead of ListenOrders
from speech_states.say import text_to_say # instead of sound_action.SpeakActionState
import understandOrders3 as parser
from geometry_msgs.msg import Pose, Point
from move_base_msgs.msg import MoveBaseGoal
from navigation_states.nav_to_poi import nav_to_poi#from pal_smach_utils.navigation.move_to_room import MoveToRoomStateMachine
#from yes or no que no se on a guardat la cris#from pal_smach_utils.speech.did_you_say_yes_or_no_sm import HearingConfirmationSM
# from gpsr.msg import order_list
from gpsrSoar.msg import gpsrActionAction
from speech_states.ask_question import AskQuestionSM
# from pal_smach_utils.grasping.initialise_and_close_grasp import InitGraspPipelineSM, CloseGraspPipelineSM

# from GenerateGoalScript import printNewGoal

# import rospy
SENTENCE_SAID = '/parsing/sentence' 
TEST = False
SKILLS = True
TEST_SENTENCE1 = "cat1Sentences/sentence"
TEST_SENTENCE2 = "cat2Sentences/sentence"
if TEST : 
    NUM_LOOPS_TODO = 203 
    NUM_LOOPS_I = 0
else: 
    NUM_LOOPS_TODO = 3
    NUM_LOOPS_I = 0
    
GRAMATICA = rospy.get_param('/parsing/GRAMATICA')
#GRAMATICA = 'robocup/minimals'#'robocup/general'

class sent():
    def __init__(self, text):
        self.text = text

class DummyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_word_heard', 'wrong_word_heard', 'aborted', 'preempted'])

    def execute(self, userdata):
        return 'correct_word_heard'
    
class ParseSentence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['i_userSaidData'],
            output_keys=['o_actionSet', 'o_confidence'])

    def execute(self, userdata):
        O = parser.orderList()
        O.parseOrders(userdata.i_userSaidData)
        if O.confidence == 'True':
            userdata.o_actionSet = O.actionSet
            return 'succeeded'
        else:
            print 'parsing returned with ' + str(O.confidence) + ' confidence. \nPlease, repeat the sentence again. \n'
            return 'aborted'


class sentence_solved(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
            input_keys=['i_actionSet'],
            output_keys=['o_stencenceSolved'])

    def execute(self,userdata):
        for command in userdata.i_actionSet:
            userdata.o_stencenceSolved = 'I understood that I should %s %s %s %s \n' % (
                command.action,
                command.location,
                command.person,
                command.item)


class init_parameters(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
            output_keys=['o_sentence', 'o_asrLoop', 'o_asrOn'])
        self.loop= NUM_LOOPS_I
    def execute(self, userdata):
        if TEST == True:
            if self.loop <= 140:
                aux = TEST_SENTENCE1+str(self.loop)
            else:
                aux = TEST_SENTENCE2+str(self.loop-140)                
            rospy.logwarn("--------- " + aux + " ---------")
            self.loop = self.loop + 1        
            userdata.o_sentence = rospy.get_param(aux)#SENTENCE_SAID)#'/parsing/sentence')
        else:        
            userdata.o_sentence = rospy.get_param(SENTENCE_SAID)#'/parsing/sentence')
        
        userdata.o_asrLoop = rospy.get_param('/parsing/ASR_LOOP')
        userdata.o_asrOn = rospy.get_param('/parsing/ASR_ON')
        return 'succeeded'


class check_AsrOn(smach.State): #Per implementar de debo (utilitzarem i i_asrOn)
    def __init__(self):
        smach.State.__init__(self, outcomes=['ASR_ON', 'ASR_OFF', 'aborted'],
            input_keys=['i_sentence', 'i_asrOn'],
            output_keys=['o_sentence'])

    def execute(self,userdata):
        check = userdata.i_asrOn
        if check == 'False':
            userdata.o_sentence = sent(userdata.i_sentence)
            return 'ASR_OFF'
        elif check == 'True':
            return 'ASR_ON'
        else:
            print 'Invalid "ASR_ON" value ' + str(check) + '\nMust be "True" or "False"'
            return 'aborted'

class announce_sentence_understood(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
            input_keys=['o_actionSet'],
            output_keys=['tts_text'])

    def execute(self,userdata):
        announced = "I understood that I should: \n"
        print('INSIDE CALLBACK')
        for command in userdata.o_actionSet:
            announced = announced + "%s %s %s %s \n" % (
                command.action,
                command.location,
                command.person,
                command.item)
        userdata.tts_text = announced
        return 'succeeded'


class check_AsrLoop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['LOOP_ON', 'LOOP_OFF', 'aborted'],
            input_keys=['i_asrLoop'])
        self.loop_i = NUM_LOOPS_I
        self.max_loop = NUM_LOOPS_TODO


    def execute(self,userdata):
        check = userdata.i_asrLoop
        if check == False or self.loop_i >= self.max_loop:
            return 'LOOP_OFF'
        elif check == True:
	    self.loop_i += 1
            return 'LOOP_ON'
        else:
            print 'Invalid "LOOP_ON" value ' + str(check) + '\nMust be "True" or "False"'
            return 'aborted'

# class generateGoals(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, smach.State.__init__(self, outcomes=[succeeded, aborted],
#             input_keys=['i_actionSet']))
#     def execute(self, userdata):
#         i=0
#         for goal in userdata:
#             i=i+1
#             name = 'goal-test' + i
#             printNewGoal(oaction=goal.action, oitem=goal.item, operson=goal.person, olocation=goal.location, goalfile=name)


# class PublishOrders(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=[succeeded, aborted],
#             input_keys=['i_actionSet'])
#     def execute(self, userdata):
#         # try:
#             pub = rospy.Publisher('nlp', order_list)
#             pub.publish(userdata.i_actionSet)
#             print 'publication succeeded'
#             return succeeded
#         # except:
#             print 'publication aborted'
#             return aborted


class gpsrOrders(smach.StateMachine):

    def __init__(self):
        checkAsrLoopState = check_AsrLoop()
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])
        self.referee_position = None
        #rospy.init_node("SM_GPSR_ORDERS")
        with self:

            self.userdata.tts_text=''
            self.userdata.tts_lang=''
            self.userdata.tts_wait_before_speaking=0            
            self.userdata.referee = 'referee'
            self.userdata.grammar_name=''
            
            '''
            smach.StateMachine.add("START_GRASP_PROTOCOL",
                                   InitGraspPipelineSM(),
                                   transitions={succeeded: 'init_SM',
                                                aborted: aborted,
                                                preempted: aborted})
            '''
            
            def move_to_caller_goal_cb(userdata, nav_goal):
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose.header.frame_id = "/map"
                # move_base_goal.target_pose.header.stamp = rospy.Time.now()
                if self.referee_position is None:
                    pose_detected = Pose()
                    pose_detected.position=Point(0, 0, 0)

                    pose = transform_pose(pose_detected, "/base_link", "/map")

                    move_base_goal.target_pose.pose = pose
                    
                    # userdata.out_referee_position = move_base_goal
                    self.referee_position = move_base_goal
                    print '************************************\n***********************************************\n***********  Referee Pose Stored ******************\n***************************************************\n*************************************************'
                else:
                    move_base_goal = self.referee_position

                move_base_goal.target_pose.header.stamp = rospy.Time.now()

                return move_base_goal


            smach.StateMachine.add( 
                    'init_SM',
                    init_parameters(),
                    transitions={'succeeded': 'Check_ASR'},
                    remapping={'o_sentence': 'sentence', 'o_asrOn': 'asrOn', 'o_asrLoop': 'asrLoop'})


            # smach.StateMachine.add(
            #         'MOVE_TO_REFEREE',
            #         MoveActionState(goal_cb=move_to_caller_goal_cb,
            #             output_keys=['out_referee_position']),
            #         remapping={'referee_position': 'out_referee_position'},
            #         transitions={succeeded: 'LISTEN_ORDER', aborted: 'aborted'})

            smach.StateMachine.add(
                    'Check_ASR',
                    check_AsrOn(),
                    transitions={'ASR_ON': 'ASK_QUESTION', 'ASR_OFF': 'PARSE_ORDER', 'aborted': 'PARSE_ORDER'},
                    remapping={'i_sentence': 'sentence', 'i_asrOn': 'asrOn', 'o_sentence': 'o_userSaidData'})

#             smach.StateMachine.add('GO_TO_REFEREE',
#                    nav_to_poi(),#MoveToRoomStateMachine(),
#                    transitions={'succeeded': 'LISTEN_ORDER', 'aborted': 'TELL_ABORTED_GO_TO'},
#                    remapping={'nav_to_poi_name': 'room_location'})#'room_name':'room_name'})

            smach.StateMachine.add(
                    'TELL_ABORTED_GO_TO',
                    text_to_say(text="Sorry I can't get to the initial point, referee could you come and tell me the command?",wait_before_speaking=0),
                    transitions={'succeeded': 'ASK_QUESTION'})

            smach.StateMachine.add(
                    'ASK_QUESTION',
                    AskQuestionSM(text="Give me the next order",grammar=GRAMATICA),
                    transitions={'succeeded': 'PARSE_ORDER', 'aborted': 'ASK_QUESTION'},
                    remapping={'asr_answer': 'o_userSaidData'})
 
#             smach.StateMachine.add(
#                     'LISTEN_ORDER',
#                     ListenToSM(grammar=GRAMATICA),
#                     transitions={'succeeded': 'ORDER_CONFIRMATION', 'aborted': 'LISTEN_ORDER'},
#                     remapping={'o_userSaidData_text': 'o_userSaidData'})
# 
# 
#             smach.StateMachine.add(
#                     'ORDER_CONFIRMATION',
#                     #TODO : when Criss uploads the yesorno function
#                     DummyState(),#HearingConfirmationSM(grammar_to_reset_when_finished=GRAMATICA),
#                     transitions={'correct_word_heard': 'PARSE_ORDER',
#                                  'wrong_word_heard': 'LISTEN_ORDER',
#                                  'preempted': 'preempted',
#                                  'aborted': 'LISTEN_ORDER'})

            smach.StateMachine.add(
                    'WRONG_WORD',
                    text_to_say(text="Then I missunderstood the command, could you repeat?",wait_before_speaking=0),
                    transitions={'succeeded': 'ANNOUNCE_SENTENCE_UNDERSTOOD_preparation'})

            smach.StateMachine.add(
                    'PARSE_ORDER',
                    ParseSentence(),
                    transitions={'succeeded': 'ANNOUNCE_LISTENED_SENTECE_RIGHT', 'aborted': 'ANNOUNCE_LISTENED_SENTENCE_WRONG'},
                    remapping={'o_actionSet': 'o_actionSet', 'i_userSaidData': 'o_userSaidData'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTECE_RIGHT',
                text_to_say(text="Sir yes sir. As you command Sir",wait_before_speaking=0),
                transitions={'succeeded': 'ANNOUNCE_SENTENCE_UNDERSTOOD_preparation'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTENCE_WRONG',
                text_to_say(text="I think I couldn't understand you, sir. Can you repeat the order?", wait_before_speaking=0),
                transitions={'succeeded': 'Check_ASR'})


            smach.StateMachine.add(
                'ANNOUNCE_SENTENCE_UNDERSTOOD_preparation',
                announce_sentence_understood(),
                transitions={'succeeded': 'ANNOUNCE_SENTENCE_UNDERSTOOD'})

            smach.StateMachine.add(
                'ANNOUNCE_SENTENCE_UNDERSTOOD',
                text_to_say(),# input_keys=['o_actionSet']),
                transitions={'succeeded': 'PUBLISH_ORDERS'})

            smach.StateMachine.add('PUBLISH_ORDERS',
                        SimpleActionState(
                            'gpsrSoar',
                            gpsrActionAction,
                            goal_slots=['orderList']),# exec_timeout=None),
                        transitions={'succeeded': 'Check_LOOP', 'aborted': 'TELL_ABORTED'},
                        remapping={'orderList': 'o_actionSet'})
    
            '''
            smach.StateMachine.add('STOP_GRASP_PROTOCOL',
                           CloseGraspPipelineSM(),
                           transitions={succeeded: succeeded,
                                        aborted: aborted,
                                        preempted: preempted})
            '''
                    
            smach.StateMachine.add('TELL_ABORTED',
                        text_to_say(text="Sorry I couldn't execute the command in time, please give me another one", wait_before_speaking=0),
                        transitions={'succeeded': 'Check_LOOP'})
            
            smach.StateMachine.add(
                    'Check_LOOP',
                    checkAsrLoopState,
                    transitions={'LOOP_ON': 'init_SM', 'LOOP_OFF': 'succeeded', 'aborted': 'init_SM'},#Check_ASR
                    remapping={'i_asrLoop': 'asrLoop'})
        


# class testParsing(smach.StateMachine):
#     def __init__(self):
#         smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])
# 
#         with self:
#             self.userdata.tts_text=''
#             self.userdata.tts_lang=''
#             self.userdata.grammar_name=''
#             self.userdata.tts_wait_before_speaking=0
#             
#             smach.StateMachine.add(
#                     'init_SM',
#                     init_parameters(),
#                     transitions={succeeded: 'Check_ASR'},
#                     remapping={'o_sentence': 'sentence', 'o_asrOn': 'asrOn', 'o_asrLoop': 'asrLoop'})
# 
#             smach.StateMachine.add(
#                     'Check_LOOP',
#                     check_AsrLoop(),
#                     transitions={'LOOP_ON': 'Check_ASR', 'LOOP_OFF': succeeded, aborted: aborted},
#                     remapping={'i_asrLoop': 'asrLoop'})
# 
#             smach.StateMachine.add(
#                     'Check_ASR',
#                     check_AsrOn(),
#                     transitions={'ASR_ON': 'LISTEN_ORDER', 'ASR_OFF': 'PARSE_ORDER', aborted: aborted},
#                     remapping={'i_sentence': 'sentence', 'i_asrOn': 'asrOn', 'o_sentence': 'o_userSaidData'})
# 
#             smach.StateMachine.add(
#                     'LISTEN_ORDER',
#                     ListenToSM(GRAMMAR_NAME=GRAMATICA),
#                     transitions={succeeded: 'PARSE_ORDER', aborted: 'LISTEN_ORDER'},
#                     remapping={'o_userSaidData': 'o_userSaidData'})
# 
#             smach.StateMachine.add(
#                     'PARSE_ORDER',
#                     ParseSentence(),
#                     transitions={succeeded: 'ANNOUNCE_LISTENED_SENTECE_RIGHT', aborted: 'ANNOUNCE_LISTENED_SENTENCE_WRONG'},
#                     remapping={'o_actionSet': 'o_actionSet', 'i_userSaidData': 'o_userSaidData'})
# 
#             smach.StateMachine.add(
#                 'ANNOUNCE_LISTENED_SENTECE_RIGHT',
#                 text_to_say(text="Sir yes sir. As you command Sir", wait_before_speaking=0),
#                 # call greeting movements!
#                 # call states for message pools
#                 transitions={succeeded: 'ANNOUNCE_SENTENCE_UNDERSTOOD_preparation'})
# 
#             smach.StateMachine.add(
#                 'ANNOUNCE_LISTENED_SENTENCE_WRONG',
#                 text_to_say(text="I think I couldn't understand you, sir. Can you repeat the order?", wait_before_speaking=0),
#                 transitions={succeeded: 'Check_ASR'})
#             
#             smach.StateMachine.add(
#                 'ANNOUNCE_SENTENCE_UNDERSTOOD_preparation',
#                 announce_sentence_understood(),
#                 transitions={succeeded: 'ANNOUNCE_SENTENCE_UNDERSTOOD'})
# 
#             smach.StateMachine.add(
#                 'ANNOUNCE_SENTENCE_UNDERSTOOD',
#                 text_to_say(),# input_keys=['o_actionSet']),
#                 transitions={succeeded: 'Check_LOOP'})

