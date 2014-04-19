#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
from smach_ros import ServiceState
from follow_me.msg import check_elevator
from follow_me.srv import EnableCheckElevatorRequest, EnableCheckElevator, EnableCheckElevatorResponse

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

MIN_SAMPLES_DOOR=3 # num of samples that the door have to be open, it have to be followed
CHECK_DOOR_OPEN = 100 # distance for the door open
FREQ_REVISION=0.1

'''
@ this state machine use check_elevator, it turns the enable on
@ it will return succeeded when it's in the elevator
you can pass an optional param that indicates when starts to loock
'''



class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],output_keys=['time_last_found'])
    def execute(self, userdata):
            userdata.time_last_found=rospy.Time.now()

            rospy.loginfo("i'm in dummy init var")
            return 'succeeded'
# its the tracker learn person...
        

class check_status(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IM_IN','IM_NOT_IN', 'preempted'], input_keys=['check_elevator_msg'])
    def execute(self,userdata):
        if self.preempt_requested():
           return 'preempted'
        if userdata.check_elevator_msg.elevator :
            return 'IM_IN'
        else :
            rospy.sleep(FREQ_REVISION)
            return 'IM_NOT_IN'
            


class print_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
    def execute(self,userdata):
       
        rospy.loginfo ("IT was impossible to read from topic")
        return 'succeeded'
    
class sleep_until(smach.State):
    def __init__(self,sleep):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.sleep=sleep
    def execute(self,userdata):
       
        rospy.loginfo ("I'm Sleeping")
        rospy.sleep(self.sleep)
        return 'succeeded'
class read_topic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],output_keys=['check_elevator_msg'])
        
    def execute(self,userdata):
        userdata.check_elevator_msg = rospy.wait_for_message('/check_elevator/elevator_status',
                                                            check_elevator, 60)
        rospy.loginfo (OKGREEN+"I'm reading"+ENDC)
        if self.preempt_requested():
           return 'preempted'
        return 'succeeded'



class look_for_elevator(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, sleep=1):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted'],
            input_keys=["in_learn_person"])
        self.sleep=sleep
        

        with self:
            

            def Cheack_Elevator_Start(userdata, request):
                start_request = EnableCheckElevatorRequest()
                start_request.enable=True
                return start_request
        
                            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "SLEEPS_UNTIL_ELEVATOR", 'preempted':'preempted'})

            smach.StateMachine.add('SLEEPS_UNTIL_ELEVATOR',
                                   sleep_until(self.sleep),
                                   transitions={'succeeded':'START_CHECK_ELEVATOR', 'preempted':'preempted'})
            
                        #call request of start enrollment
            smach.StateMachine.add('START_CHECK_ELEVATOR',
                                    ServiceState('/check_elevator/enable',
                                    EnableCheckElevator,
                                    request_cb = Cheack_Elevator_Start),
                                    transitions={'succeeded':'READ_TOPIC','aborted' : 'Print_error','preempted':'preempted'})
   

            smach.StateMachine.add('READ_TOPIC',
                                   read_topic(),
                                   transitions={'succeeded': 'CHECK_STATUS', 'preempted':'preempted'})            
            smach.StateMachine.add('CHECK_STATUS',
                                   check_status(),
                                   transitions={'IM_IN': 'succeeded',
                                                'IM_NOT_IN': 'READ_TOPIC', 'preempted':'preempted'})

            smach.StateMachine.add('Print_error',
                                   print_error(),
                                   transitions={'succeeded': 'READ_TOPIC', 'preempted':'preempted'})
                                            





class init_var_2nd(smach.StateMachine):
    
    def __init__(self):
            smach.StateMachine.__init__(self,
                                        outcomes=['succeeded', 'preempted'],
                                        input_keys=[],output_keys=['num_count'])
    def execute (self,userdata):
        userdata.num_count=0
        if self.preempt_requested():
            return 'preempted'
        return 'succeeded'




class check_door_status(smach.StateMachine):
    
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['DOOR','NO_DOOR', 'preempted'],input_keys=['check_elevator_msg'])

    def execute(self,userdata):
        
        # i look if the distance is OK
        if self.preempt_requested():
           return 'preempted'

        if  not userdata.check_elevator_msg.ultra_sound_door:
            return 'NO_DOOR'
        else :
            return 'DOOR'
        
class count_door(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded','not_yet', 'preempted'],
                                    input_keys=['num_count'],output_keys=['num_count'])
    def execute(self,userdata):
        userdata.num_count=userdata.num_count+1
        if self.preempt_requested():
           return 'preempted'
        if userdata.num_count>=MIN_SAMPLES_DOOR :
            return 'succeeded'
        else :
            rospy.sleep(0.2)# i sleep because i prefer to give some time to the system
            return 'not_yet'
        
        
class reset_count(smach.StateMachine):  
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'preempted'],
                                    output_keys=['num_count'])
    def execute(self,userdata):
        userdata.num_count=0
        if self.preempt_requested():
           return 'preempted'       
        return 'succeeded'
    
    
    
    
class look_for_elevator_door(smach.StateMachine):
    #Its an infinite loop looking the door

    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded', 'preempted'],input_keys=[])
        

        with self:
            


            smach.StateMachine.add('INIT_VAR',
                                   init_var_2nd(),
                                  transitions={'succeeded': "READ_AUX", 'preempted':'preempted'})
            
            smach.StateMachine.add('READ_AUX',
                                   read_topic(),
                                   transitions={'succeeded': 'CHECK_DOOR_STATUS', 'preempted':'preempted'})            
            smach.StateMachine.add('CHECK_DOOR_STATUS',
                                   check_door_status(),
                                   transitions={'DOOR': 'RESET_COUNT',
                                                'NO_DOOR': 'COUNT_DOOR', 'preempted':'preempted'})

            smach.StateMachine.add('COUNT_DOOR',
                                   count_door(),
                                   transitions={'succeeded': 'succeeded','not_yet':'READ_AUX', 'preempted':'preempted'})
            smach.StateMachine.add('RESET_COUNT',
                                   reset_count(),
                                   transitions={'succeeded': 'READ_AUX','preempted':'preempted'})





