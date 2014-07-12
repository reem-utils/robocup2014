#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLLOW_ME.PY ###
import smach
import rospy
from speech_states.say import text_to_say
"""
@author: Roger Boldu
"""
# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


from speech_states.say import text_to_say
from speech_states.listen_and_check_word import ListenWordSM
from speech_states.say_with_enable import say_with_enable
from pipol_tracker_pkg.msg import personArray,person
from util_states.topic_reader import topic_reader
from manipulation_states.move_head_form import move_head_form
from std_msgs.msg import Int32


Y_CALIBRARTION=0.5 # it calibrates the person that robot takes
X_CALIBRATION=2.3
TIME_WAIT=0.1 # this are secons


NEW_PERSON="A you are here, lets go"
# It's only becouse i can't import the file... i can't understand
class select_ID(smach.State):

    def __init__(self, pub): 
        smach.State.__init__(self, input_keys=['tracking_msg','in_learn_person'],
                             output_keys=['in_learn_person'],
                             outcomes=['succeeded','aborted', 'preempted'])
        self.pub=pub
    def execute(self, userdata):
        userdata.in_learn_person=None
        person_detect=userdata.tracking_msg
        per_aux=person()
        #per_follow=[]
        minx=X_CALIBRATION
        found =False

        for person_aux in person_detect.peopleSet :
            if (-Y_CALIBRARTION<person_aux.y<Y_CALIBRARTION) and ((person_aux.targetStatus & person.VISUALLY_CONFIRMED) or (person_aux.targetStatus & person.OCCLUDDED)):
                
                if person_aux.x<minx :
                    minx=person_aux.x
                    userdata.in_learn_person=person_aux
                    found=True
                
        
        if found :
            self.pub.publish(userdata.in_learn_person.targetId)
            rospy.loginfo(OKGREEN+"i have learned the person whit  ID  :  " 
                              + str(userdata.in_learn_person)+ENDC)
            return 'succeeded'

        
        userdata.in_learn_person=None
        return 'aborted'
    
    
    

class init_var(smach.State):

    def __init__(self): 
        smach.State.__init__(self, input_keys=[],
                             output_keys=['time_init'],
                             outcomes=['succeeded','aborted', 'preempted'])

    def execute(self, userdata):
        userdata.time_init=rospy.get_rostime()
        return 'succeeded'
class wait_time(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,input_keys=[],output_keys=[],outcomes=['succeeded','aborted', 'preempted'])
    def execute(self,userdata):
        rospy.sleep(TIME_WAIT)
        return 'succeeded'

class LearnPersonRandom(smach.StateMachine):
    def __init__(self,feedback=True):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'],
                                    output_keys=['standard_error','in_learn_person'])
        self.feedback = feedback
        self.follow_pub = rospy.Publisher('/follow_me/id', Int32, latch=True)  
        with self:
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            self.userdata.standard_error='OK'
            


            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                   transitions={'succeeded': 'DEFAULT_POSITION',
                                                'aborted': 'DEFAULT_POSITION'})
            
            smach.StateMachine.add('DEFAULT_POSITION',
                                   move_head_form("center","up"),
                                   transitions={'succeeded': 'WAIT_TIME','aborted':'WAIT_TIME'})
            
            smach.StateMachine.add('WAIT_TIME',
                       wait_time(),
                       transitions={'succeeded': 'READ_TRACKER_TOPIC',
                                    'aborted': 'READ_TRACKER_TOPIC'})

#TODO:: aborted->CONTROL_TIME
            smach.StateMachine.add('READ_TRACKER_TOPIC',
                                   topic_reader(topic_name='/pipol_tracker_node/peopleSet',
                                                topic_type=personArray,topic_time_out=60),
                                   transitions={'succeeded':'SELECT_ID',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'topic_output_msg': 'tracking_msg'})
           
            
            # it learns the person that we have to follow
            smach.StateMachine.add('SELECT_ID',
                                   select_ID(self.follow_pub),
                                   transitions={'succeeded': 'NEW_PERSON',
                                                'aborted': 'WAIT_TIME'})
            
            smach.StateMachine.add('NEW_PERSON',
                       say_with_enable(text=NEW_PERSON,enable=self.feedback),
                       transitions={'succeeded': 'succeeded','preempted':'succeeded', 'aborted':'succeeded'})

