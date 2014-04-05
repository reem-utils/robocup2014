#! /usr/bin/env python
# vim: expandtab ts=4 sw=4

"""
@author: Roger Boldu
"""

import smach
import rospy

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from speech_states.say import text_to_say
from speech_states.listen_to import  ListenToSM
#from learn_person import LearnPerson

from follow_me_1st import follow_me_1st
from follow_me_2nd import follow_me_2nd
from follow_me_3rd import follow_me_3rd
 
FOLLOW_GRAMMAR_NAME = 'robocup/followme'

START_FOLLOW_FRASE = "Ok, I'll follow you wherever you want. Please come a bit closer if you are too far, then Please stay still while I learn how you are."
LEARNED_PERSON_FRASE = "Let's go buttercup."

class FollowMeInit_dummy(smach.State):
    def __init__(self): 
      smach.State.__init__(self, input_keys=[],
                           outcomes=['succeeded'])
    
    def execute(self,userdata):
        rospy.loginfo (OKGREEN +"I'm followin dummy state   "  + ENDC)
        return 'succeeded'
    
class follow_me_3rd_dummy(smach.State):
    def __init__(self): 
      smach.State.__init__(self, input_keys=[],
                           outcomes=['succeeded'])
    
    def execute(self,userdata):
        rospy.loginfo (OKGREEN +"I'm followin dummy 3 state   "  + ENDC)
        return 'succeeded'


class wait_for_stard(smach.State):

    def __init__(self): 
        smach.State.__init__(self, input_keys=['asr_userSaid'],
                             outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        
        
        if userdata.asr_userSaid=="Follow me":
            return 'succeded' # todo locks to .gram
        else :
            print (FAIL +"I listen diferent a diferent thing   " +  str(userdata.asr_userSaid) + ENDC)
            return 'aborted'
        return 'succeeded'


    
#Main
class FollowMe(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:


            smach.StateMachine.add('INIT_FOLLOW',
                                   FollowMeInit_dummy(),
                                   transitions={'succeeded': 'FOLLOW_ME_1rst'})

            smach.StateMachine.add('FOLLOW_ME_1rst',
                                   follow_me_1st(),
                                   transitions={'ELEVATOR':'FOLLOW_ME_2nd', 'LOST':'FOLLOW_ME_1rst'})
            
            # in this state i will wait that the door it comes open
            smach.StateMachine.add('FOLLOW_ME_2nd',
                                   follow_me_2nd(),
                                   transitions={'succeeded': 'succeeded', 'aborted':'aborted'})
#             smach.StateMachine.add('FOLLOW_ME_3rd',
#                                    follow_me_3rd_dummy(),
#                                    transitions={'succeeded': 'succeeded'})