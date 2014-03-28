import roslib
roslib.load_manifest('gpsr')
import smach
from smach_ros import SimpleActionState
import rospy

from pal_smach_utils.utils.global_common import succeeded, preempted, aborted, transform_pose
from pal_smach_utils.speech.sm_listen_orders import ListenOrders  # check if the SM is correctly defined
from pal_smach_utils.speech.sound_action import SpeakActionState
import understandOrders3 as parser
from geometry_msgs.msg import Pose, Point
from move_base_msgs.msg import MoveBaseGoal
from pal_smach_utils.navigation.move_to_room import MoveToRoomStateMachine
from pal_smach_utils.speech.did_you_say_yes_or_no_sm import HearingConfirmationSM
# from gpsr.msg import order_list
from gpsrSoar.msg import gpsrActionAction
from pal_smach_utils.navigation.move_to_room import MoveToRoomStateMachine
# from pal_smach_utils.grasping.initialise_and_close_grasp import InitGraspPipelineSM, CloseGraspPipelineSM

# from GenerateGoalScript import printNewGoal

# import rospy

NUM_LOOPS_TODO = 3
NUM_LOOPS_I = 0
GRAMATICA = 'robocup/gentest'

class sent():
    def __init__(self, text):
        self.text = text

class ParseSentence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[succeeded, aborted, preempted],
            input_keys=['i_userSaidData'],
            output_keys=['o_actionSet', 'o_confidence'])

    def execute(self, userdata):
        O = parser.orderList()
        O.parseOrders(userdata.i_userSaidData)
        if O.confidence == 'True':
            userdata.o_actionSet = O.actionSet
            return succeeded
        else:
            print 'parsing returned with ' + str(O.confidence) + ' confidence. \nPlease, repeat the sentence again. \n'
            return aborted


class sentence_solved(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[succeeded],
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
        smach.State.__init__(self, outcomes=[succeeded],
            output_keys=['o_sentence', 'o_asrLoop', 'o_asrOn'])
    def execute(self, userdata):
        userdata.o_sentence = rospy.get_param('/parsing/sentence')
        userdata.o_asrLoop = rospy.get_param('/parsing/ASR_LOOP')
        userdata.o_asrOn = rospy.get_param('/parsing/ASR_ON')
        return succeeded


class check_AsrOn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ASR_ON', 'ASR_OFF', aborted],
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
            return aborted

class check_AsrLoop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['LOOP_ON', 'LOOP_OFF', aborted],
            input_keys=['i_asrLoop'])
        self.loop_i = 0
        self.max_loop = 3


    def execute(self,userdata):
        check = userdata.i_asrLoop
        if check == False or self.loop_i >= self.max_loop:
            return 'LOOP_OFF'
        elif check == True:
	    self.loop_i += 1
            return 'LOOP_ON'
        else:
            print 'Invalid "LOOP_ON" value ' + str(check) + '\nMust be "True" or "False"'
            return aborted

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
        smach.StateMachine.__init__(self, [succeeded, preempted, aborted])
        self.referee_position = None
        with self:

            '''smach.StateMachine.add("START_GRASP_PROTOCOL",
                                   InitGraspPipelineSM(),
                                   transitions={succeeded: 'init_SM',
                                                aborted: aborted,
                                                preempted: aborted})'''
            
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
                    transitions={succeeded: 'Check_ASR'},
                    remapping={'o_sentence': 'sentence', 'o_asrOn': 'asrOn', 'o_asrLoop': 'asrLoop'})

            # smach.StateMachine.add(
            #         'MOVE_TO_REFEREE',
            #         MoveActionState(goal_cb=move_to_caller_goal_cb,
            #             output_keys=['out_referee_position']),
            #         remapping={'referee_position': 'out_referee_position'},
            #         transitions={succeeded: 'LISTEN_ORDER', aborted: 'aborted'})

            
            smach.StateMachine.add(
                    'Check_LOOP',
                    checkAsrLoopState,
                    transitions={'LOOP_ON': 'Check_ASR', 'LOOP_OFF': succeeded, aborted: 'Check_ASR'},
                    remapping={'i_asrLoop': 'asrLoop'})

            smach.StateMachine.add(
                    'Check_ASR',                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                    check_AsrOn(),
                    transitions={'ASR_ON': 'LISTEN_ORDER', 'ASR_OFF': 'PARSE_ORDER', aborted: 'PARSE_ORDER'},
                    remapping={'i_sentence': 'sentence', 'i_asrOn': 'asrOn', 'o_sentence': 'o_userSaidData'})

            self.userdata.referee = 'referee'

            smach.StateMachine.add('GO_TO_REFEREE',
                   MoveToRoomStateMachine(),
                   transitions={succeeded: 'LISTEN_ORDER', aborted: 'TELL_ABORTED_GO_TO'},
                   remapping={'room_name': 'referee', 'room_location': 'room_location'})

            smach.StateMachine.add(
                'TELL_ABORTED_GO_TO',
                SpeakActionState(text="Sorry I can't get to the initial point, referee could you come and tell me the command?"),
                transitions={succeeded: 'LISTEN_ORDER'})

            smach.StateMachine.add(
                    'LISTEN_ORDER',
                    ListenOrders(GRAMMAR_NAME=GRAMATICA),
                    transitions={succeeded: 'ORDER_CONFIRMATION', aborted: 'LISTEN_ORDER'},
                    remapping={'o_userSaidData_text': 'o_userSaidData'})


            smach.StateMachine.add(
                    'ORDER_CONFIRMATION',
                    HearingConfirmationSM(grammar_to_reset_when_finished=GRAMATICA),
                    transitions={'correct_word_heard': 'PARSE_ORDER',
                                 'wrong_word_heard': 'LISTEN_ORDER',
                                 preempted: preempted,
                                 aborted: 'LISTEN_ORDER'},
                    remapping={'in_message_heard': 'o_userSaidData'})

            smach.StateMachine.add(
                    'WRONG_WORD',
                    SpeakActionState(text="Then I missunderstood the command, could you repeat?"),
                    transitions={succeeded: 'ANNOUNCE_SENTENCE_UNDERSTOOD'})

            smach.StateMachine.add(
                    'PARSE_ORDER',
                    ParseSentence(),
                    transitions={succeeded: 'ANNOUNCE_LISTENED_SENTECE_RIGHT', aborted: 'ANNOUNCE_LISTENED_SENTENCE_WRONG'},
                    remapping={'o_actionSet': 'o_actionSet', 'i_userSaidData': 'o_userSaidData'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTECE_RIGHT',
                SpeakActionState(text="Sir yes sir. As you command Sir"),
                # call greeting movements!
                # call states for message pools
                transitions={succeeded: 'ANNOUNCE_SENTENCE_UNDERSTOOD'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTENCE_WRONG',
                SpeakActionState(text="I think I couldn't understand you, sir. Can you repeat the order?"),
                transitions={succeeded: 'Check_ASR'})

            def announce_sentence_understood(userdata):
                announced = "I understood that I should: \n"
                for command in userdata.o_actionSet:
                    announced = announced + "%s %s %s %s \n" % (
                        command.action,
                        command.location,
                        command.person,
                        command.item)
                return announced

            smach.StateMachine.add(
                'ANNOUNCE_SENTENCE_UNDERSTOOD',
                SpeakActionState(text_cb=announce_sentence_understood, input_keys=['o_actionSet']),
                transitions={succeeded: 'PUBLISH_ORDERS'})

            # smach.StateMachine.add(
            #     'GENERATE_GOALS',
            #     generateGoals(),
            #     transitions={succeeded: 'PUBLISH_ORDERS'})

            smach.StateMachine.add(
                    'PUBLISH_ORDERS',
                    SimpleActionState(
                        'gpsrSoar',
                        gpsrActionAction,
                        goal_slots=['orderList']),
                    transitions={succeeded: 'Check_LOOP', aborted: 'TELL_ABORTED'},
                    remapping={'orderList': 'o_actionSet'})

            '''smach.StateMachine.add('STOP_GRASP_PROTOCOL',
                                   CloseGraspPipelineSM(),
                                   transitions={succeeded: succeeded,
                                                aborted: aborted,
                                                preempted: preempted})'''
            
            smach.StateMachine.add(
                'TELL_ABORTED',
                SpeakActionState(text="Sorry I couldn't execute the command in time, please give me another one"),
                transitions={succeeded: 'Check_LOOP'})

        


class testParsing(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, [succeeded, preempted, aborted])

        with self:

            smach.StateMachine.add(
                    'init_SM',
                    init_parameters(),
                    transitions={succeeded: 'Check_ASR'},
                    remapping={'o_sentence': 'sentence', 'o_asrOn': 'asrOn', 'o_asrLoop': 'asrLoop'})

            smach.StateMachine.add(
                    'Check_LOOP',
                    check_AsrLoop(),
                    transitions={'LOOP_ON': 'Check_ASR', 'LOOP_OFF': succeeded, aborted: aborted},
                    remapping={'i_asrLoop': 'asrLoop'})

            smach.StateMachine.add(
                    'Check_ASR',
                    check_AsrOn(),
                    transitions={'ASR_ON': 'LISTEN_ORDER', 'ASR_OFF': 'PARSE_ORDER', aborted: aborted},
                    remapping={'i_sentence': 'sentence', 'i_asrOn': 'asrOn', 'o_sentence': 'o_userSaidData'})

            smach.StateMachine.add(
                    'LISTEN_ORDER',
                    ListenOrders(GRAMMAR_NAME=GRAMATICA),
                    transitions={succeeded: 'PARSE_ORDER', aborted: 'LISTEN_ORDER'},
                    remapping={'o_userSaidData': 'o_userSaidData'})

            smach.StateMachine.add(
                    'PARSE_ORDER',
                    ParseSentence(),
                    transitions={succeeded: 'ANNOUNCE_LISTENED_SENTECE_RIGHT', aborted: 'ANNOUNCE_LISTENED_SENTENCE_WRONG'},
                    remapping={'o_actionSet': 'o_actionSet', 'i_userSaidData': 'o_userSaidData'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTECE_RIGHT',
                SpeakActionState(text="Sir yes sir. As you command Sir"),
                # call greeting movements!
                # call states for message pools
                transitions={succeeded: 'ANNOUNCE_SENTENCE_UNDERSTOOD'})

            smach.StateMachine.add(
                'ANNOUNCE_LISTENED_SENTENCE_WRONG',
                SpeakActionState(text="I think I couldn't understand you, sir. Can you repeat the order?"),
                transitions={succeeded: 'Check_ASR'})

            def announce_sentence_understood(userdata):
                announced = "I understood that I should: \n"
                for command in userdata.o_actionSet:
                    announced = announced + "%s %s %s %s \n" % (
                        command.action,
                        command.location,
                        command.person,
                        command.item)
                return announced

            smach.StateMachine.add(
                'ANNOUNCE_SENTENCE_UNDERSTOOD',
                SpeakActionState(text_cb=announce_sentence_understood, input_keys=['o_actionSet']),
                transitions={succeeded: 'Check_LOOP'})

