#! /usr/bin/env python
"""
@author: Roger Boldu
"""
import rospy
import smach

from follow_operator import FollowOperator



class check_elevator(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'])

    def execute(self, userdata):
        rospy.sleep(2)
        rospy.loginfo("i'm in dummy elevator state")
       # userdata.standard_error="Dummy"
        return 'succeeded'
'''     
class Follow_operator(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted','preempted'])

    def execute(self, userdata):
        rospy.sleep(2)
       # userdata.standard_error="Dummy"
        return 'succeeded'
'''
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
    def __init__(self, distToHuman=0.9):
        smach.Concurrence.__init__(self,outcomes=['succeeded', 'preempted', 'aborted'],
                                   default_outcome='succeeded',input_keys=["in_learn_person"])
        
       # rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", distToHuman)
    
        with self:
    
            
            smach.Concurrence.add('FOLLOW_OPERATOR',
                            FollowOperator())
            #This it will return if it's in the elevator, and if it's in the elevator
            # it have to say: i'm in the elevator
            # it have to sent a goal in a less distance of the operator
            smach.Concurrence.add('CHECK_ELEVATOR',
                            check_elevator())
                  
               
            
