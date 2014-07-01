#! /usr/bin/env python

import rospy
import actionlib
import smach
from smach_ros import SimpleActionState
import smach_ros

from blort_msgs.msg import RecognizeAction, RecognizeGoal, RecognizeResult 

from object_recognition_msgs.msg import ObjectType

objectDetect_topic = '/blort_tracker/recognize_object'

class Prepare_data(smach.State):
    def __init__(self, object_detect_name):
        smach.State.__init__(self, 
                             outcomes = ['succeeded', 'aborted'], 
                             input_keys = ['object_name'], 
                             output_keys = ['object_name'])
        self.object_detect_name = object_detect_name
        
    def execute(self, userdata):
        if not self.object_detect_name and not userdata.object_name:
            rospy.logerr("No Object to Detect")
            return 'aborted'
        
        userdata.object_name = self.object_detect_name if self.object_detect_name else userdata.object_name
        
        return 'succeeded'

class prepare_object_detection_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succeeded', 'aborted'], 
                             input_keys = ['object_detection_goal','object_name'], 
                             output_keys = ['object_detection_goal'])
    def execute(self, userdata):
        
        userdata.object_detection_goal = RecognizeGoal()
        userdata.object_detection_goal.refine_pose_time = 4.0
        
        if type(userdata.object_name) is str:
            object_to_detect_type = ObjectType()
            object_to_detect_type.key = userdata.object_name
            userdata.object_detection_goal.objects.append(object_to_detect_type)
        
            rospy.loginfo("Goal that is sending:: " + str(userdata.object_detection_goal))
        
        elif type(userdata.object_name) is list:
            for o_name in userdata.object_name:
                object_to_detect_type = ObjectType()
                object_to_detect_type.key = o_name
                userdata.object_detection_goal.objects.append(object_to_detect_type)
       
        
        return 'succeeded'

class object_detect_sm(smach.StateMachine):
    def __init__(self, object_to_detect_name=None):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['object_name'],
                                    output_keys=['object_pose'])
        with self:
            smach.StateMachine.add('Prepare_data',
                                   Prepare_data(object_to_detect_name),
                                   transitions={'succeeded':'Prepare_goal'})
            smach.StateMachine.add('Prepare_goal',
                                   prepare_object_detection_goal(),
                                   transitions={'succeeded':'Object_detect'})
            
            def object_detect_result_cb(self, status, object_detect_result):
                print ('Object Detected: ' + str(object_detect_result))
                self.object_pose = object_detect_result
                return 'succeeded'
                
            smach.StateMachine.add('Object_detect', 
                                SimpleActionState(objectDetect_topic,
                                                   RecognizeAction,
                                                   result_cb=object_detect_result_cb,
                                                   goal_key='object_detection_goal',
                                                   output_keys=['object_pose','standard_error']), 
                                transitions={'succeeded':'succeeded', 'aborted':'aborted'})
            
            
def main():
    rospy.init_node('search_object_node')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
 
    with sm:
        sm.userdata.object_name = 'Barritas' 
        
        smach.StateMachine.add('Search_Object',
                            object_detect_sm(),
                            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
 
    sis = smach_ros.IntrospectionServer(
        'robocup_instrospection', sm, '/SM_ROOT')
    sis.start()
 
    sm.execute()
 
    #rospy.spin()
    sis.stop()
 
if __name__ == '__main__':
    main()