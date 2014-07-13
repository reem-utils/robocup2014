'''
Created on 12/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy

from grasp_order import GraspOrder
from deliver_order import DeliverOrder
from navigation_states.nav_to_poi import nav_to_poi

class ExecuteOrder(smach.StateMachine):
    """
    Executes a SM that execute for all the orders in cocktail Party.
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['object_name', 'name_face', 'list_orders', 'loop_iterations'], 
                                            output_keys=[])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            
            # Grasp Order
            smach.StateMachine.add(
                 'grasp_order',
                 GraspOrder(),
                 transitions={'succeeded': 'go_to_party', 'aborted': 'aborted'}) 
            
            # Go to the party room
            smach.StateMachine.add(
                'go_to_party',
                nav_to_poi('party_room'),
                transitions={'succeeded': 'deliver_order', 'aborted': 'go_to_party', 
                'preempted': 'preempted'}) 

            # Deliver the order
            smach.StateMachine.add(
                'deliver_order',
                DeliverOrder(),
                transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
  
            