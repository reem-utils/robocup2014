#! /usr/bin/env python
"""
Created on 14/06/2014

@author: Cristina De Saint Germain
"""
import rospy
import smach

from speech_states.listen_and_check_word import ListenWordSM_Concurrent
from navigation_states.nav_to_poi import nav_to_poi


ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

# 
class prepareData(smach.State):
    
    def __init__(self, poi_name, listen_word):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                            input_keys=['nav_to_poi_name', 'word_to_listen'], output_keys=['nav_to_poi_name', 'word_to_listen'])
        self.poi_name = poi_name
        self.listen_word = listen_word
        
    def execute(self, userdata):
           
        if not self.poi_name and not userdata.nav_to_poi_name:
            rospy.logerr("Poi_name isn't set")
            return 'aborted'

        if not self.listen_word and not userdata.word_to_listen:
            rospy.logerr("listen_word isn't set")
            return 'aborted'
        
        #Priority in init
        userdata.nav_to_poi_name = self.poi_name if self.poi_name else userdata.nav_to_poi_name   
        userdata.word_to_listen = self.listen_word if self.listen_word else userdata.word_to_listen 
        
        return 'succeeded'
    
    
# gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # terminate all running states if walk_to_poi finished with outcome succeeded
    if outcome_map['go_to_poi'] == 'succeeded':
        rospy.loginfo(OKGREEN + "I got the poi that I except" + ENDC)
        return True

    # terminate all running states if BAR finished
    if outcome_map['listen_word'] == 'succeeded':
        rospy.loginfo(OKGREEN + "I listen the word that I expect" + ENDC)
        return True
    
    # in all other case, just keep running, don't terminate anything
    return False

def out_cb_follow(outcome_map):
    
    if outcome_map['go_to_poi'] == 'succeeded':
        return 'NAV'    
    elif outcome_map['listen_word'] == 'succeeded':
        return 'LISTEN'    
    

class Go_Poi_Listen_Word(smach.StateMachine):
    """
    Executes a SM that goes to a poi and while it's going,
    the robot can wait for a listen command.  
    """
    def __init__(self, poi='', word=''):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted',
                                                    'aborted'],
                                    input_keys=['nav_to_poi_name','word_to_listen'])
        
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.word_to_listen=None
            self.userdata.nav_to_poi_name = None
            
            smach.StateMachine.add('PrepareData',
               prepareData(poi, word),
               transitions={'succeeded':'go_and_listen', 'aborted':'aborted'})
            
            sm=smach.Concurrence(outcomes=['NAV','LISTEN','preempted'],
                                    default_outcome='NAV',input_keys=["nav_to_poi_name",
                                                                           'word_to_listen'],
                                    child_termination_cb = child_term_cb,
                                    outcome_cb=out_cb_follow,output_keys=[])
            
            with sm:
    
                smach.Concurrence.add('go_to_poi', nav_to_poi())

                smach.Concurrence.add('listen_word', ListenWordSM_Concurrent())
            

                
            
            smach.StateMachine.add('go_and_listen', sm,
                                     transitions={'NAV':'succeeded',
                                                 'LISTEN':'aborted', 'preempted':'succeeded'})
            
          
                  
               
            
