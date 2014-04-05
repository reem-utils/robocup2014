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
from util_states import topic_reader

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'


'''
@ this state machine use check_elevator, it turns the enable on
@ it will return succeeded when it's in the elevator
you can pass an optional param that indicates when starts to loock
'''



class init_var(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['time_last_found'])
    def execute(self, userdata):
            userdata.time_last_found=rospy.Time.now()
            rospy.sleep(1)
            rospy.loginfo("i'm in dummy init var")
            return 'succeeded'
# its the tracker learn person...
        

class check_status(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IM_IN','IM_NOT_IN'], input_keys=['check_elevator_msg'])
    def execute(self,userdata):
        if userdata.check_elevator_msg.elevator :
            return 'IM_IN'
        else :
            return 'IM_NOT_IN'
            


class print_error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self,userdata):
       
        rospy.loginfo ("IT was impossible to read from topic")
        return 'succeeded'
    
class sleep_until(smach.State):
    def __init__(self,sleep):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.sleep=sleep
    def execute(self,userdata):
       
        rospy.loginfo ("I'm Sleeping")
        rospy.sleep(self.sleep)
        return 'succeeded'
class read_aux(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],output_keys=['check_elevator_msg'])
        
    def execute(self,userdata):
        userdata.check_elevator_msg = rospy.wait_for_message('/check_elevator/elevator_status', check_elevator, 60)
        rospy.loginfo ("I'm reading")
        return 'succeeded'



class look_for_elevator(smach.StateMachine):
    #Its an infinite loop track_Operator

    def __init__(self, sleep=1):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=["in_learn_person"])
        self.sleep=sleep
        

        with self:
            

            def Cheack_Elevator_Start(userdata, request):
                start_request = EnableCheckElevatorRequest()
                start_request.enable=True
                return start_request
            def Cheack_Elevator_Stop(userdata, request):
                start_request = EnableCheckElevatorRequest()
                start_request.enable=False
                return start_request
        
                            
            smach.StateMachine.add('INIT_VAR',
                                   init_var(),
                                  transitions={'succeeded': "SLEEPS_UNTIL_ELEVATOR"})

            smach.StateMachine.add('SLEEPS_UNTIL_ELEVATOR',
                                   sleep_until(self.sleep),
                                   transitions={'succeeded':'START_CHECK_ELEVATOR'})
            
                        #call request of start enrollment
            smach.StateMachine.add('START_CHECK_ELEVATOR',
                                    ServiceState('/check_elevator/enable',
                                    EnableCheckElevator,
                                    request_cb = Cheack_Elevator_Start),
                                    transitions={'succeeded':'READ_AUX','aborted' : 'Print_error','preempted':'Print_error'})
   

            smach.StateMachine.add('READ_AUX',
                                   read_aux(),
                                   transitions={'succeeded': 'CHECK_STATUS'})            
            smach.StateMachine.add('CHECK_STATUS',
                                   check_status(),
                                   transitions={'IM_IN': 'succeeded',
                                                'IM_NOT_IN': 'READ_AUX'})

            smach.StateMachine.add('Print_error',
                                   print_error(),
                                   transitions={'succeeded': 'READ_AUX'})
                                            
