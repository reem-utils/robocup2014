#! /usr/bin/env python
 
import roslib
roslib.load_manifest('gpsr')
import rospy
import smach
import smach_ros
#from pal_smach_utils.utils.global_common import succeeded, preempted, aborted
 

 
 
def main():
    rospy.init_node('gpsr_test_')
    # Import needs to be after init_node for some reason
    from sm_gpsr_orders import gpsrOrders
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
 
    with sm:
        smach.StateMachine.add('UNDERSTANDING_TEST',
                            gpsrOrders(),
                            transitions={
                            'succeeded': 'succeeded', 'aborted': 'aborted'})
 
    sis = smach_ros.IntrospectionServer(
        'understanding_introspection', sm, '/SM_ROOT')
    sis.start()
 
    sm.execute()
 
    rospy.spin()
    sis.stop()
 
if __name__ == '__main__':
    main()
