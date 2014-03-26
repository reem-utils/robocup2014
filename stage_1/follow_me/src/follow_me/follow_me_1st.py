#! /usr/bin/env python

import rospy
import smach




class Listen_Comands(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'])

    def execute(self, userdata):
        rospy.sleep(2)
       # userdata.standard_error="Dummy"
        return 'succeeded'
     
class Follow_operator(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'])

    def execute(self, userdata):
        rospy.sleep(2)
       # userdata.standard_error="Dummy"
        return 'succeeded'

#Defining the state Machine of follow_me 1st part
class follow_me_1st(smach.Concurrence):
    """
    Executes a SM that do the first part of the follow_me.
    This part consist in follow the operator,
    2 person it will cross, one of them it will stops for 3 seconds 
    this part of the follow_me it will be finished when the robot 
    arrives to the "small room"
    
    It creates a concurrence state machine for:
        Listen
        Follow
    
    """
    def __init__(self, distToHuman=0.9, state_machine_name="restaurant"):
        smach.Concurrence.__init__(self,outcomes=['succeeded', 'preempted', 'aborted'],default_outcome='succeeded',input_keys=["id_learn_person"])
        rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", distToHuman)
    
        with self:
    
            
            smach.Concurrence.add('FOLLOW_OPERATOR',
                            Follow_operator(),
                            remapping={"id_learn_person": "id_learn_person"})
            
            #fumada... no vec perque ho fan aixi....
            # jo el que vec ésque podem fer-ho aixi i un cop senti enter ja puc salta a la seguent fase
            smach.Concurrence.add('LISTEN_COMMANDS',
                            Listen_Comands())
                  
               
            
