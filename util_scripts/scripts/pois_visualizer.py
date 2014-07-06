#!/usr/bin/env python
"""
Created on 9/06/14

@author: Sam Pfeiffer

File to show POIs and it's names in Rviz
"""

# system stuff
import sys
import copy

# ROS stuff
import rospy
import rosparam
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import Header
from tf import transformations
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

global id_marker
id_marker = 99

POI_POSES_TOPIC = '/POI_poses'
POI_TEXTMARKERS_TOPIC = '/POI_names'

def create_marker_from_poi(namespace_poi, poi):
    """Returns a text Marker from the POI"""
    # POI looks like:
    # ['submap_0', 'point_room_three', -0.809, 6.411, -1.441]
    m = Marker()
    m.action = m.ADD
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    global id_marker
    m.id = id_marker
    id_marker += 1
    m.pose.position.x = poi[2]
    m.pose.position.y = poi[3]
    m.pose.position.z = 0.5
    m.pose.orientation.w = 1.0
    m.text = namespace_poi + "/" +  poi[1]
    m.type = m.TEXT_VIEW_FACING
    m.scale.z = 0.2 # This is the size of the text
    m.color.r = 1.0
    m.color.a = 1.0
    return m

def create_pose_from_poi(poi):
    """Returns a Pose from the POI.
    POIs are specified like X, Y, RotationYaw"""
    # POI looks like:
    # ['submap_0', 'point_room_three', -0.809, 6.411, -1.441]
    poi_pose = Pose()
    poi_pose.position.x = poi[2]
    poi_pose.position.y = poi[3]
    quat = quaternion_from_euler(0.0, 0.0, poi[4])
    poi_pose.orientation = Quaternion(*quat)
    return poi_pose

def create_marker_array(markers):
    """Given a list of markers create a MarkerArray"""
    ma = MarkerArray()
    for marker in markers:
        ma.markers.append(marker)
    return ma

def create_pose_array(poses):
    """Given a pois_dict create a pose_array with the POIs poses"""
    pois_pa = PoseArray()
    pois_pa.header.frame_id = "map"
    pois_pa.header.stamp = rospy.Time.now()
    for pose in poses:
        pois_pa.poses.append(pose)
    return pois_pa

def get_params(ns):
    """Get the params of each POI and it's location"""
    pois_dict = rosparam.get_param(ns)
    # Looks like:
#     {'numberOfSubMaps': 1,
#  'poi': {'submap_0': {'avoid_that': ['submap_0',
#     'avoid_that',
#     -7.298,
#     5.911,
#     -2.252],
#    'fetch_and_carry': ['submap_0', 'fetch_and_carry', -2.0, -2.0, 0],
# ... etc, it has submembers called poi, numberOfSubMaps and vo, at least, using mmap
    poi_dicts_to_remove = []
    for poi_dict_name in pois_dict:
        if poi_dict_name != "poi": # and poi_dict_name != "vo": # we only want the pois on a subspace called poi or vo # we dont really want vo
            poi_dicts_to_remove.append(poi_dict_name)
    for poi_dict_name in poi_dicts_to_remove:
        pois_dict.pop(poi_dict_name)
    return pois_dict

def usage():
    print "Usage:"
    print sys.argv[0] + " namespace_pois\n"
    print "For example: " + sys.argv[0] + " mmap"
    
if __name__ == '__main__':
    rospy.init_node("show_pois")
    rospy.sleep(0.3)
    if len(sys.argv) != 2:
        usage()
        exit(0)
    param_namespace = sys.argv[1]
    
    pois_dicts = get_params(param_namespace)
    
    tmp_list_poses = []
    tmp_list_markers = []
#     print "pois_dicts: ",
#     print pois_dicts
    for poi_dict_name in pois_dicts:
#         print "poi_dict: ",
#         print pois_dicts.get(poi_dict_name)
        curr_dict = pois_dicts.get(poi_dict_name)
        for poi_dict in curr_dict:
#             print "poi_dict: ",
#             print curr_dict.get(poi_dict)
            curr_poi_dict = curr_dict.get(poi_dict)
            for poi in curr_poi_dict:
#                 print "poi:"
#                 print curr_poi_dict.get(poi)
                curr_poi = curr_poi_dict.get(poi)
                print "Adding poi: " + str(curr_poi)
                tmp_list_poses.append(create_pose_from_poi(curr_poi))
                tmp_list_markers.append(create_marker_from_poi(poi_dict_name, curr_poi))

    pois_pa = create_pose_array(tmp_list_poses)
    pois_ma = create_marker_array(tmp_list_markers)

#     print "POIs pose array: ",
#     print pois_pa
#     print "POIs marker array: ",
#     print pois_ma
    
    pa_pub = rospy.Publisher(POI_POSES_TOPIC, PoseArray)
    ma_pub = rospy.Publisher(POI_TEXTMARKERS_TOPIC, MarkerArray)

    rospy.loginfo("Publishing " + str(len(pois_ma.markers)) + " POIs")
    rospy.loginfo("At topics: " + POI_POSES_TOPIC + " " + POI_TEXTMARKERS_TOPIC)
    while not rospy.is_shutdown():
        pa_pub.publish(pois_pa)
        ma_pub.publish(pois_ma)
        rospy.sleep(0.3)
        
    