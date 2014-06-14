#!/usr/bin/env python

"""
@author:  Chang Long Zhu
@email: changlongzj@gmail.com
@author: Sammy Pfeiffer

19 Feb 2014
"""


#import rospy
import rospy
import smach
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from speech_states.say import text_to_say

# Constants
NAVIGATION_TOPIC_NAME = '/move_base'



class createNavGoal(smach.State):
    def __init__(self, frame_id):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
							input_keys=['nav_to_coord_goal', 'nav_to_coord_frame'], output_keys=['navigation_goal','standard_error'])
        self.frame_id = frame_id
        
    def execute(self, userdata):
        self.frame_id = userdata.nav_to_coord_frame if userdata.nav_to_coord_goal else self.frame_id
        rospy.loginfo("Frame from -- nav_to_coord_ud -- :: " + self.frame_id)
        nav_goal = self.create_nav_goal(userdata.nav_to_coord_goal[0], 
									userdata.nav_to_coord_goal[1], 
									userdata.nav_to_coord_goal[2],
                                    userdata.nav_to_coord_goal[3],
                                    userdata.nav_to_coord_goal[4],
                                    userdata.nav_to_coord_goal[5])
        
        userdata.navigation_goal = nav_goal
        return 'succeeded'
 
    def create_nav_goal(self, x, y, z, roll, pitch, yaw):
        """Create a MoveBaseGoal with x, y position and yaw rotation (in radians).
        Returns a MoveBaseGoal"""
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = self.frame_id # Note: the frame_id must be map
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = z # z must be 0.0 (no height in the map)
         
        # Orientation of the robot is expressed in the yaw value of euler angles
        quat = quaternion_from_euler(roll, pitch, yaw) # roll, pitch, yaw
        mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

        return mb_goal

class nav_to_coord_ud(smach.StateMachine):
    """
    Navigate to given map coords.

    This SM navigates to a given coordenates using the parameter: "nav_to_coord_goal"
    This input data should be a (x,y,yaw) point
    With frame_id param you can indicate if the coord are from the map (frame_id = /map)
    or from the robot (frame_id = /base_link). By default it is /map. 

    @input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
    rotation in radians
    @output_keys: standard_error string representing the possible error
    """

    
    def __init__(self,frame_id='/map'):
        """
        Constructor for nav_to_coord.
        """
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=['nav_to_coord_goal', 'nav_to_coord_frame'],
                                 output_keys=['standard_error'])

        with self: 

            smach.StateMachine.add('CreateNavGoal',
                                   createNavGoal(frame_id),
                                   transitions={'succeeded':'MoveRobot', 'aborted':'aborted'})
            
            def move_res_cb(userdata, result_status, result):

                if result_status != 3: # 3 == SUCCEEDED
                    if result_status == 4: 
                        userdata.standard_error = "Aborted navigation goal (maybe we didn't get there?)"
                        rospy.loginfo(userdata.standard_error)
                    elif result_status == 5: # We havent got a rejected yet, maybe never happens
                        userdata.standard_error = "Rejected navigation goal (maybe the goal is outside of the map or in a obstacle?)"
                        rospy.loginfo(userdata.standard_error)
                    elif result_status == 2:
                        return 'preempted'
                    return 'aborted'
                else:
                    userdata.standard_error = "OK"
                    return 'succeeded'
        
            
            smach.StateMachine.add('MoveRobot', 
								SimpleActionState(NAVIGATION_TOPIC_NAME,
                                                   MoveBaseAction,
                                                   goal_key='navigation_goal',
                                                   input_keys=['standard_error'],
                                                   output_keys=['standard_error'],
                                                   result_cb=move_res_cb), 
								transitions={'succeeded':'succeeded', 'aborted':'Aborting_text', 'preempted':'preempted'})
            
            smach.StateMachine.add('Aborting_text',
                                   text_to_say('I cannot reach the location, sorry.'),
                                   transitions={'succeeded':'succeeded', 'aborted':'aborted'})
def main():
    rospy.loginfo('Go POi Node')
    rospy.init_node('go_poi')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:      
        sm.userdata.nav_to_coord_goal = [1.0,0.0,0.0]
        smach.StateMachine.add(
            'Go_TO_PI',
            nav_to_coord(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()
