#! /usr/bin/env python
'''
@author: Sam Pfeiffer
'''
# System stuff
import copy
import math
import sys

# ROS stuff
import rospy
import tf
from tf.transformations import quaternion_from_euler

# Msgs
from pipol_tracker_pkg.msg import person, personArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from util_states.math_utils import *
from control_msgs.msg import PointHeadActionGoal

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

PEOPLE_TRACKER_TOPIC = '/pipol_tracker_node/peopleSet'
POINT_HEAD_TOPIC = '/head_controller/point_head_action/goal'


def normalize_2d_vector(vector):
    """Normalize a 2d vector, make it unitary
    vector is a Point
    @return Point normalized"""
    magnitude = math.sqrt(vector.x**2 + vector.y**2)
    return Point(vector.x/magnitude, vector.y/magnitude, 0.0)


def substract_points(p1, p2):
    """Being p1 and p2 Point()s
    compute the substraction of p1 - p2
    and
    @return Point resulting point."""
    substracted_point = Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)
    return substracted_point


def get_2d_angle(x1, x2, y1, y2):
    """Return the angle beween two 2d vectors.
    @return double radians angle between the vectors"""
    return math.atan2(x2 - x1, y2 - y1)


def compute_orientation(pose1, pose2):
    """Compute yaw orientation from the pose of the tracked person
    and the current robot pose, both in the same reference frame.
    @return Quaternion with the yaw angle of the vector pointing from pose1 to pose2"""
    dummy_point = Point(1.0, 0.0, 0.0)
    substracted_point = substract_points(pose1.pose.position, pose2.pose.position)
    normalized_point = normalize_2d_vector(substracted_point)
    yaw_angle = get_2d_angle(dummy_point.x, normalized_point.x, dummy_point.y, normalized_point.y)
    if math.degrees(yaw_angle) < -90.0:
        yaw_angle = (math.pi - yaw_angle) * -1
    rospy.loginfo("yaw_angle is in degrees: " + str(math.degrees(yaw_angle)))
    tmp_quat = quaternion_from_euler(0.0, 0.0, -yaw_angle) # must give the inverse angle
    return Quaternion(*tmp_quat)


def createGoal(x, y, distanceToHuman):
    print "x: " + str(x) + " y: " + str(y) + " dist human: " + str(distanceToHuman)
    new_pose = Pose()
    new_pose.position.x = x
    new_pose.position.y = y
    
    unit_vector = normalize_vector(new_pose.position)
    position_distance = vector_magnitude(new_pose.position)
#     rospy.loginfo(" Position data from Reem to person:")
#     rospy.loginfo(" Position vector : " + str(new_pose.position))
#     rospy.loginfo(" Unit position vector : " + str(unit_vector))
#     rospy.loginfo(" Position vector distance : " + str(position_distance))
    
    distance_des = 0.0 #TODO: I DON? UNDERSTAND allwas it will be a movment
    
    if position_distance >= distanceToHuman: 
        distance_des = position_distance - distanceToHuman
    
    else:
        rospy.loginfo(OKGREEN+" Person too close => not moving, just rotate"+ENDC)
    #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
    alfa = math.atan2(new_pose.position.y, new_pose.position.x)
    dist_vector = multiply_vector(unit_vector, distance_des)
    
    tmp_quat = quaternion_from_euler(0.0, 0.0, alfa)
    new_pose.orientation = Quaternion(*tmp_quat)
    
    ps_goal = PoseStamped()
    ps_goal.header.frame_id = "base_link"
    ps_goal.header.stamp = rospy.Time.now() + rospy.Duration(4.0)
    new_pose.position.x = dist_vector.x
    new_pose.position.y = dist_vector.y
    ps_goal.pose = new_pose
    ps_goal.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    print "\n goal is:"
    print ps_goal
    return ps_goal




class FollowPipolTrackerIdWithHead():
    """
    Subscribe to face detections
    and publish a personArray and also
    a PointStamped with the 3D pose of the person.
    """
    
    def __init__(self):
        rospy.loginfo("Setting publisher to '" + POINT_HEAD_TOPIC + "'")
        self.point_head_pub = rospy.Publisher(POINT_HEAD_TOPIC, PointHeadActionGoal)

        rospy.loginfo("Setting subscriber to '" + PEOPLE_TRACKER_TOPIC + "'")
        self.pipol_tracker_sub = rospy.Subscriber(PEOPLE_TRACKER_TOPIC, personArray, self.people_tracker_cb, queue_size=1)
        self.last_pipol_msg = None
        self.new_msg = False
        rospy.loginfo("Waiting for first pipol tracker msg...")
        while self.last_pipol_msg == None:
            rospy.sleep(0.1)
        rospy.loginfo("Got one, node prepared!")
        self.curr_goal_to_send = None

    def people_tracker_cb(self, data):
        """cb for pipol tracker topic"""
        self.last_pipol_msg = data
        self.new_msg = True

    def run(self, id_to_follow):
        """Publishing in topics (depending on rate of publication on face detections)"""
        id_is_in_msg = True
        max_lost_iterations = 30
        curr_lost_iterations = 0
        dist_to_human = 0.6
        last_time = rospy.Time.now()
        while not rospy.is_shutdown() and id_is_in_msg:
            if self.new_msg:
                self.new_msg = False
                found_id_in_msg = False
                for person in self.last_pipol_msg.peopleSet:
                    #print "person is: " + str(person)
                    if person.targetId == id_to_follow:
                        found_id_in_msg = True
                        phg = PointHeadActionGoal()
                        phg.goal.min_duration = rospy.Duration(1.0) # adapt for as far as the detection is??
                        phg.goal.target.header.frame_id = "base_link"
                        phg.goal.target.header.stamp = rospy.Time.now()
                        phg.goal.target.point.x = person.x
                        phg.goal.target.point.y = person.y
                        phg.goal.target.point.z = 1.7 # always 1.7meters of Z
                        phg.goal.pointing_axis.x = 1.0
                        phg.goal.pointing_frame = 'head_mount_xtion_rgb_frame'

                        #Publish
                        if rospy.Time.now() - last_time > rospy.Duration(0.5):
                            self.point_head_pub.publish(phg)
                            last_time = rospy.Time.now()
#                     else:
#                         print "person.targetId != id_to_follow"
#                         print str(person.targetId) + " != " + str(id_to_follow)
                if not found_id_in_msg: # If we lost the id stop
                    curr_lost_iterations += 1
                    if curr_lost_iterations > max_lost_iterations:
                        id_is_in_msg = False
            else:
                rospy.sleep(0.1)
            
        rospy.logerr("We lost the id! Stopping the tracking!")



if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "Usage: " + sys.argv[0] + " <id_to_follow>"
        print "Example:\n"
        print sys.argv[0] + " 12"
        exit(0)
    rospy.init_node('follow_vision_raw')
    rospy.sleep(0.5)  # Give a moment to init_node to do its job
    rospy.loginfo("Initializing follow id")
    follow_person = FollowPipolTrackerIdWithHead()
    rospy.loginfo("Running, going to follow id: " + str(sys.argv[1]))
    follow_person.run(int(sys.argv[1]))
