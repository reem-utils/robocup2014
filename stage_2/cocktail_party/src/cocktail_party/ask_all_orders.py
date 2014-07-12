#! /usr/bin/env python
'''
Created on 10/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy

from speech_states.say import text_to_say
from ask_order import AskOrder

NUMBER_OF_ORDERS = 3

class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['loop_iterations'],
                                output_keys=['standard_error', 'loop_iterations', "did_pick",'delete_database'])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_ORDERS:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.loop_iterations = userdata.loop_iterations + 1
            userdata.delete_database=False
            return 'succeeded'

class process_info(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], 
                                input_keys=['name_face', 'object_name', 'loop_iterations', 'list_orders'],
                                output_keys=['list_orders'])

    def execute(self, userdata):
        userdata.list_orders.append([userdata.name_face, userdata.object_name])
        rospy.loginfo(str(userdata.list_orders))
        return 'succeeded'
  
 
class AskAllOrders(smach.StateMachine):
    """
    Executes a SM that ask for all the orders in cocktail Party.
        
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
                                            input_keys=[], 
                                            output_keys=['list_orders'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.loop_iterations = 1
            self.userdata.list_orders = []
            self.userdata.delete_database=True            # Ask Order
            smach.StateMachine.add(
                 'ask_order',
                 AskOrder(),
                 transitions={'succeeded': 'process_info', 'aborted': 'aborted'}) 
            
            # Process the info [name, object_name]
            smach.StateMachine.add(
                'process_info',
                process_info(),
                transitions={'succeeded': 'check_loop', 'aborted': 'aborted'})
            
            # End of loop?
            smach.StateMachine.add(
                'check_loop',
                checkLoop(),
                transitions={'succeeded': 'ask_order', 'aborted': 'aborted', 
                'preempted': 'preempted', 'end':'succeeded'}) 
                       
def main():
    rospy.init_node('cocktail_order')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'cocktail_orders',
            AskAllOrders(),
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
