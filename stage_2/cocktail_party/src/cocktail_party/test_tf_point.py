#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, PointStamped, Point
import tf

rospy.init_node('teetetetst')
rospy.sleep(0.5)
rospy.loginfo("Getting a TransformListener...")
tf_listener = tf.TransformListener()
rospy.sleep(2)
# To initialize the tf listener
latest_common_tf_time = tf_listener.getLatestCommonTime("base_link", "head_mount_xtion_rgb_optical_frame")

ps = PointStamped()
ps.header.frame_id = 'head_mount_xtion_rgb_optical_frame'
ps.header.stamp = rospy.Time.now()
ps.point = Point()

# transform the pose of the face detection to base_link to compute on that frame
transform_ok = False
while not transform_ok: # this is ugly as is polling to TF... but works
    try:
        transformed_point = tf_listener.transformPoint('base_link', ps)
        transform_ok = True
    except tf.ExtrapolationException:
        rospy.logwarn("Exception on transforming transformed_point... trying again.")
        ps.header.stamp = rospy.Time.now()
        rospy.sleep(0.3)

print "done"
