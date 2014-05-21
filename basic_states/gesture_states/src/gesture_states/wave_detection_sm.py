#! /usr/bin/env python
"""
Created on 21/05/14

@author: Chang Long Zhu Jin
@mail: changlongzj@gmail.com
"""

import rospy
import smach
import tf
import tf.transformations as TT
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point, Quaternion
from util_states.topic_reader import topic_reader
from pal_vision_msgs.msg import Gesture
import numpy
import copy

GESTURE_TOPIC = '/head_mount_xtion/gestures'
final_frame_id = 'base_link'
PUB_1_TOPIC = '/given_pose'
PUB_2_TOPIC = '/transformed_pose'


class TransformGesture(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                             input_keys=['topic_output_msg'],
                             output_keys=['wave_position','standard_error'])


        rospy.loginfo("Subscribing to '" + GESTURE_TOPIC + "'")
        #self.face_sub = rospy.Subscriber(GESTURE_TOPIC, Gesture, self.gesture_cb)
        # Debugging publishers
        self.pub1 = rospy.Publisher(PUB_1_TOPIC, PoseStamped)
        self.pub2 = rospy.Publisher(PUB_2_TOPIC, PoseStamped)

        rospy.loginfo("Getting a TransformListener...")
        self.tf_listener = tf.TransformListener()
        #self.gesture_data = self.userdata.gesture_data.topic_output_msg
        
    def execute(self, userdata): 
        self.gesture_data = userdata.topic_output_msg
        rospy.loginfo("Got gesture:\n" + str(self.gesture_data))
        #self.gesture_data = Gesture()
        # Create a PoseStamped to debug
        gesture_ps = PoseStamped(
                                   header = Header(
                                                 stamp = self.gesture_data.header.stamp,
                                                 frame_id = self.gesture_data.header.frame_id  # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                   pose = Pose(
                                             position = self.gesture_data.position3D, 
                                             orientation = Quaternion(w = 1.0)
                                             )
                                   )
        self.pub1.publish(gesture_ps)
        
        # Create PointStamped to transform it
        gesture_pointstamped = PointStamped(
                                            header = Header(
                                                 stamp = rospy.Time(0),  # This is to get the latest available transform
                                                 frame_id = self.gesture_data.header.frame_id  # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                            point = self.gesture_data.position3D
                                            )

        try:
            transformed_gesture_pointstamped = self.tf_listener.transformPoint(final_frame_id, gesture_pointstamped)
            rospy.loginfo("Transformed pose:\n" + str(transformed_gesture_pointstamped))
            # Debugging PoseStamped
            pub2_ps = PoseStamped(
                               header = Header(
                                                 stamp = transformed_gesture_pointstamped.header.stamp,
                                                 frame_id = transformed_gesture_pointstamped.header.frame_id # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                pose = Pose(
                                             position = transformed_gesture_pointstamped.point, 
                                             orientation = Quaternion(w = 1.0)
                                             )
                              )
            self.pub2.publish(pub2_ps)
            userdata.wave_position = transformed_gesture_pointstamped

            return 'succeeded'
        except:
            rospy.loginfo('Transform Exception -- Trying again')
            #userdata.wave_position = PointStamped()
            rospy.sleep(1)

            transformed_gesture_pointstamped = self.tf_listener.transformPoint(final_frame_id, gesture_pointstamped)
            userdata.wave_position = transformed_gesture_pointstamped
            rospy.loginfo("Transformed pose 22222:\n" + str(transformed_gesture_pointstamped))
            # Debugging PoseStamped
            pub2_ps = PoseStamped(
                               header = Header(
                                                 stamp = transformed_gesture_pointstamped.header.stamp,
                                                 frame_id = transformed_gesture_pointstamped.header.frame_id # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                pose = Pose(
                                             position = transformed_gesture_pointstamped.point, 
                                             orientation = Quaternion(w = 1.0)
                                             )
                              )
            self.pub2.publish(pub2_ps)
            userdata.wave_position = transformed_gesture_pointstamped
            if transformed_gesture_pointstamped is None:
                return 'aborted'
            else:
                return 'succeeded'

        


class GestureDetection(smach.StateMachine):
    """
        GestureDetection - It is a State Machine that subscibes to the topic '/head_mount_xtion/gestures', using the topic_reader()
        This Topic gives the 3D position of the wave gesture in the frame 'head_mount_xtion_depth_optical_frame', 
        so after reading the topic, a TF has to be calculated.

        Input Keys:
            None
        Output Keys:
            @key wave_position: A PointStamped point referenced to /base_link
            @key standard_error: A base error to inform.

        Required Parameters: 
            None

    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                 input_keys=[],
                                 output_keys=['wave_position','standard_error'])
        with self:
            smach.StateMachine.add(
                'Gesture_Topic_Reader',
                topic_reader(topic_name=GESTURE_TOPIC, topic_type=Gesture, topic_time_out=60.0, blocked=False),
                transitions={'succeeded':'TransformGesture', 'preempted':'Gesture_Topic_Reader', 'aborted':'Gesture_Topic_Reader'})

            smach.StateMachine.add(
                'TransformGesture',
                TransformGesture(),
                transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})



def main():
    rospy.loginfo('Wave Detection Node')
    rospy.init_node('wave_detection_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'gesture_state',
            GestureDetection(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})

    sm.execute()
    rospy.spin()

if __name__=='__main__':
    main()













