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

from follow_me_init import FollowMeInit
from follow_me_1st import follow_me_1st
from follow_me_2nd import follow_me_2nd
from follow_me_3rd import follow_me_3rd
 
FOLLOW_GRAMMAR_NAME = 'robocup/followme'

LIFT_TEXT="this lift is really small"
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
            rospy.loginfo (FAIL +"I listen diferent a diferent thing   " +  str(userdata.asr_userSaid) + ENDC)
            return 'aborted'
        return 'succeeded'


    
#Main
class FollowMe(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:  
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standard_error='OK'
            self.userdata.standard_error='OK'
            
            smach.StateMachine.add('INIT_FOLLOW',
                                   FollowMeInit(),
                                   transitions={'succeeded': 'FOLLOW_ME_1rst', 
                                                'preempted':'FOLLOW_ME_1rst', 'aborted':'FOLLOW_ME_1rst'})

            smach.StateMachine.add('FOLLOW_ME_1rst',
                                   follow_me_1st(),
                                   transitions={'succeeded':'TEXT', 'aborted':'FOLLOW_ME_1rst','preempted':'TEXT',
                                                'operator_say_out':'FOLLOW_ME_3rd'})
            
            # in this state i will wait that the door it comes open
            smach.StateMachine.add('TEXT',
                       text_to_say(LIFT_TEXT),
                       transitions={'succeeded': 'FOLLOW_ME_2nd','aborted':'aborted'})
            
            
            smach.StateMachine.add('FOLLOW_ME_2nd',
                                   follow_me_2nd(),
                                   transitions={'succeeded': 'FOLLOW_ME_3rd', 'aborted':'aborted'})


            smach.StateMachine.add('FOLLOW_ME_3rd',
                                    follow_me_3rd(),
                                    transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})