#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 12/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy

from execute_order import ExecuteOrder
from navigation_states.nav_to_poi import nav_to_poi

NUMBER_OF_ORDERS = 3

class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['loop_iterations'],
                                output_keys=['standard_error', 'loop_iterations', "did_pick"])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_ORDERS:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'
  
class prepare_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['list_orders', 'object_name'],
                                output_keys=['object_name', 'name_face', 'name'])

    def execute(self, userdata):
        for item in userdata.list_orders:
            userdata.object_name.append(item[1])
        
        return 'succeeded'

        
class ExecuteAllOrders(smach.StateMachine):
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
                                            input_keys=['list_orders'], 
                                            output_keys=['list_orders'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 1
            self.userdata.object_name = []
            self.userdata.list_orders = [['hannah', 'red bull'], ['rebecca', 'water'], ['andrew', 'Cocacola']]
            
            # Go to the storage_room
            smach.StateMachine.add(
                'go_to_storage',
                nav_to_poi("storage_room"),
                transitions={'succeeded': 'prepare_recognize', 'aborted': 'prepare_recognize', 
                'preempted': 'preempted'}) 
            
            # Prepare next order
            smach.StateMachine.add(
                'prepare_recognize',
                prepare_recognize(),
                transitions={'succeeded': 'execute_order', 'aborted': 'aborted', 
                'preempted': 'preempted'})
            
            # Execute Order
            smach.StateMachine.add(
                 'execute_order',
                 ExecuteOrder(),
                 transitions={'succeeded': 'check_loop', 'aborted': 'aborted'}) 
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'execute_order', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'succeeded'}) 
                       
                        
def main():
    rospy.init_node('cocktail_execute')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'cocktail_execute',
            ExecuteAllOrders(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    
    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
#     sis = smach_ros.IntrospectionServer(
#         'basic_functionalities_introspection', sm, '/BF_ROOT')
#     sis.start()

    sm.execute()

    rospy.spin()
 #   sis.stop()


if __name__ == '__main__':
    main()
           