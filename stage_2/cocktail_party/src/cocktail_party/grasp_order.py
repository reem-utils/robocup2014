'''
Created on 12/07/2014

@author: Cristina De Saint Germain
'''

import smach
import rospy

from speech_states.say import text_to_say
from navigation_states.nav_to_coord import nav_to_coord
from geometry_msgs.msg import PoseStamped
from manipulation_states.play_motion_sm import play_motion_sm
from hri_states.recognize_object_and_pick import RecObjectAndPick
from object_grasping_states.pick_object_sm import pick_object_sm
from manipulation_states.ask_give_object_grasping import ask_give_object_grasping

NUMBER_OF_TRIES = 3
    
class dummy_recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=['object_position','pose_to_place','nav_to_poi_name'], 
            output_keys=['object_position','pose_to_place', 'nav_to_poi_name'])

    def execute(self, userdata):
        
        userdata.object_position = PoseStamped()
        userdata.object_position.header.frame_id = "base_link"
        userdata.object_position.pose.position.x = 0.5
        userdata.object_position.pose.position.z = 1.0
        userdata.object_position.pose.orientation.w = 1.0
         
        rospy.sleep(5)
        return 'succeeded'

class checkLoop(smach.State):
    def __init__(self):
        rospy.loginfo("Entering loop_test")
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted', 'end'], 
                                input_keys=['loop_iterations', 'did_unk'],
                                output_keys=['standard_error', 'loop_iterations', "did_unk"])

    def execute(self, userdata):
        
        if userdata.loop_iterations == NUMBER_OF_TRIES:
            return 'end'
        else:
            rospy.loginfo(userdata.loop_iterations)
            userdata.standard_error='OK'
            userdata.did_unk = True
            userdata.loop_iterations = userdata.loop_iterations + 1
            return 'succeeded'

class change_did_pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=["did_pick"],
                                output_keys=["did_pick"])
        
    def execute(self, userdata):
    
        userdata.did_pick = False
        return 'succeeded'

class GraspOrder(smach.StateMachine):
    """
    Executes a SM that execute one order in cocktail Party.
        
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
                                    input_keys=['object_name', 'name'],
                                    output_keys=['did_pick'])

        with self:
            # We must initialize the userdata keys if they are going to be accessed or they won't exist and crash!
            self.userdata.try_iterations = 1
            self.userdata.did_pick = True
            
            # State that prepare the information for pick - Mock recognition 
            smach.StateMachine.add(
                'object_recognition',
                dummy_recognize(),
                transitions={'succeeded': 'play_motion_grasp', 'aborted': 'play_motion_grasp', 
                'preempted': 'preempted'}) 
            
            # Recognize and pick object if found
            smach.StateMachine.add(
                'recognize_object_and_pick',
                RecObjectAndPick(),
                transitions={'succeeded': 'succeeded', 
                             'fail_grasp':'Grasp_fail_Ask_Person',
                             'fail_recognize': 'try_again_recognition'})
    
        # Recognize Fail Part
            # We don't recognized the object
            smach.StateMachine.add(
                'try_again_recognition',
                checkLoop(),
                transitions={'succeeded': 'recognize_object_and_pick', 'aborted': 'recognize_object_and_pick', 
                'preempted': 'preempted', 'end':'Grasp_fail_Ask_Person'}) 
            
        #Grasp Mock Part 
            # Home position
            smach.StateMachine.add(
                'play_motion_grasp',
                play_motion_sm('home'),
                transitions={'succeeded': 'say_grasp_object', 'preempted':'say_grasp_object', 
                             'aborted':'play_motion_grasp'}) 
            
            # Say grasp object
            smach.StateMachine.add(
                 'say_grasp_object',
                 text_to_say("I'm going to grasp the object"),
                 transitions={'succeeded': 'grasp_object', 'aborted': 'grasp_object'})
             
            # Grasp the object
            smach.StateMachine.add(
                'grasp_object',
                pick_object_sm(),
                transitions={'succeeded': 'succeeded', 'aborted': 'Grasp_fail_Ask_Person', 
                'preempted': 'preempted'})    
     
        # We ask for the object            
            # Ask for grasp object
            smach.StateMachine.add(
                'Grasp_fail_Ask_Person',
                ask_give_object_grasping(),
                remapping={'object_to_grasp':'object_name'},
                transitions={'succeeded':'Rest_arm', 'aborted':'Rest_arm', 'preempted':'Rest_arm'})
            
            smach.StateMachine.add(
                 'Rest_arm',
                 play_motion_sm('rest_object_right'),
                 transitions={'succeeded':'change_did_pick', 'aborted':'change_did_pick', 'preempted':'change_did_pick'})
      
            smach.StateMachine.add(
                 'change_did_pick',
                 change_did_pick(),
                 transitions={'succeeded':'succeeded', 'aborted':'succeeded', 'preempted':'preempted'})
        
  