import rospy

from math_utils import normalize_vector2D, vector_magnitude2D, multiply_vector2D, substract_vector, add_vectors
from geometry_msgs.msg import Pose

def pose_at_distance(pose,distance):
    """
    Returns a pose that has the same orientation as the original
    but the position is at a distance from the original.
    Very usefull when you want to mantain a distance from an object. 
    """
    
    rospy.loginfo("pose at distance")
    unit_vector = normalize_vector2D(pose.position)
    k = vector_magnitude2D(pose.position)
    distance_des = k - distance
    rospy.loginfo(k)
    rospy.loginfo(distance_des)
    dist_vector = multiply_vector2D(unit_vector, distance_des)
    new_pose = Pose()
    new_pose.position = dist_vector
    new_pose.orientation = pose.orientation
    rospy.loginfo(new_pose.position)
    return new_pose
# vim: expandtab ts=4 sw=4


def pose_at_distance2(pose1,pose2,distance):
    """
    Returns a pose that has the same orientation as the original
    but the position is at a distance from the original.
    Very usefull when you want to mantain a distance from an object. 
    """
                
    new_pose2 = Pose()
    new_pose2.position = substract_vector(pose2.position, pose1.position)
    new_pose2 = pose_at_distance(new_pose2,distance)
    
    new_pose2.position = add_vectors(new_pose2.position,pose1.position)
    return new_pose2
# vim: expandtab ts=4 sw=4
