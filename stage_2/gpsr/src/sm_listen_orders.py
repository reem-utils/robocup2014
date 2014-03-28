import roslib
roslib.load_manifest('pal_smach_utils')
import smach

from pal_smach_utils.utils.global_common import succeeded, preempted, aborted
from pal_smach_utils.utils.topic_reader import TopicReaderState
from gramar_state import GrammarState

from pal_interaction_msgs.msg import *
from pal_interaction_msgs.srv import *





class ListenOrders(smach.StateMachine):

    def __init__(self, GRAMMAR_NAME):
        smach.StateMachine.__init__(self, [succeeded,preempted,aborted],
            output_keys = ['o_userSaidData'])


        with self:
            smach.StateMachine.add(
                    'ENABLE_GRAMMAR', 
                    GrammarState(GRAMMAR_NAME, enabled = True),
                    transitions = {succeedeeded: 'HEAR_COMMAND'})

            smach.StateMachine.add(
                    'HEAR_COMMAND', 
                    TopicReaderState(topic_name='usersaid',msg_type=asrresult,timeout=None), 
                    transitions = {aborted: 'HEAR_COMMAND', succeeded:'PRINT_MESSAGE',preempted:preempted},
                    remapping = {'message':'o_userSaidData'})
            
            smach.StateMachine.add( 'DISABLE_GRAMMAR',
                    GrammarState(GRAMMAR_NAME, enabled=False),
                    transitions = {succeeded: succeeded})
