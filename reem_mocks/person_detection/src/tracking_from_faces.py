#! /usr/bin/env python
'''
@author: Sam Pfeiffer
'''
# System stuff
import copy
import math

# ROS stuff
import rospy
import tf
from tf.transformations import quaternion_from_euler

# Msgs
from pal_detection_msgs.msg import FaceDetection, FaceDetections
from follow_me.msg import tracker_people, person, personArray
from pal_detection_msgs.msg import FaceDetections, Detections2d, FaceDetection, Detection2d
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


fakepositionx=0
fakepositiony=2
fakeorientation=0
information=True
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

FACES_TOPIC = '/pal_face/faces'
PEOPLE_TRACKER_TOPIC = '/people_tracker_node/peopleSet'
TRACKED_POSESTAMPED_TOPIC = '/move_base/follow_goal'


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


class TrackFromFaces():
    """
    Subscribe to face detections
    and publish a personArray and also
    a PointStamped with the 3D pose of the person.
    """
    
    def __init__(self):
        # Get subscriber and publishers and stuff
        rospy.loginfo("Subscribing to '" + FACES_TOPIC + "'")
        self.face_sub = rospy.Subscriber(FACES_TOPIC, FaceDetections, self.face_cb)

        rospy.loginfo("Setting publisher to '" + PEOPLE_TRACKER_TOPIC + "'")
        self.tracker_pub = rospy.Publisher(PEOPLE_TRACKER_TOPIC, personArray)

        rospy.loginfo("Getting a TransformListener...")
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Setting publisher to '" + TRACKED_POSESTAMPED_TOPIC + "'")
        self.tracked_posestamped_pub = rospy.Publisher(TRACKED_POSESTAMPED_TOPIC, PoseStamped)

        # Init class variables
        self.last_faces_msg = None
        self.new_face_msg = False

    def face_cb(self, data):
        self.last_faces_msg = data
        if len(self.last_faces_msg.faces) > 0:
            self.new_face_msg = True

    def create_tracker_msgs(self):
        current_msg = copy.deepcopy(self.last_faces_msg)
        per = person()  # person msg
        person_arr = personArray()
        pose_stamped_first_face = None
        #rospy.loginfo("waiting for transform...")
        latest_common_tf_time = self.tf_listener.getLatestCommonTime("base_link", current_msg.header.frame_id)
        #self.tf_listener.waitForTransform("odom", current_msg.header.frame_id, latest_common_tf_time, rospy.Duration(1.0)) # Not needed
        #rospy.loginfo("got a transform!")

        if len(current_msg.faces) > 0 : # should always be filled, but just in case
            target_id = 1
            for face in current_msg.faces:
                # Temporal PoseStamped for transforming into odom frame the 3d pose of the face detection
                face_ps = PoseStamped()
                face_ps.header.frame_id = current_msg.header.frame_id
                face_ps.header.stamp = latest_common_tf_time #rospy.Time.now() #latest_common_tf_time
                face_ps.pose.position = face.position
                face_ps.pose.orientation.w = 1.0

                current_robot_ps = PoseStamped()
                current_robot_ps.header.frame_id = 'base_link'
                current_robot_ps.header.stamp = rospy.Time.now()
                current_robot_ps.pose.position = Point(0.0, 0.0, 0.0)
                current_robot_ps.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

                # transform the pose of the face detection to base_link to compute on that frame
                transform_ok = False
                while not transform_ok: # this is ugly as is polling a lot to TF... but works
                    try:
                        transformed_baselink_pose = self.tf_listener.transformPose("base_link", face_ps)
                        transform_ok = True
                    except tf.ExtrapolationException:
                        rospy.logwarn("Exception on transforming transformed_pose... trying again.")
                        face_ps.header.stamp = rospy.Time.now()

                # Compute the orientation of the tracking goal in base_link
                transformed_baselink_pose.pose.orientation = compute_orientation(transformed_baselink_pose, current_robot_ps)
                transformed_baselink_pose.pose.position.z = 0.0
                
                # transform the pose to odom for convenience (follow planner needs odom)
#                 transform_ok = False
#                 while not transform_ok:
#                     try:
#                         transformed_odom_pose = self.tf_listener.transformPose("odom", transformed_baselink_pose)
#                         transform_ok = True
#                     except tf.ExtrapolationException:
#                         rospy.logwarn("Exception on transforming transformed_pose... trying again.")
#                         transformed_baselink_pose.header.stamp = rospy.Time.now()

                if not pose_stamped_first_face:
                    #pose_stamped_first_face = transformed_odom_pose
                    pose_stamped_first_face = transformed_baselink_pose
                per.targetId = target_id
                target_id += 1
                per.x = transformed_baselink_pose.pose.position.x
                per.y = transformed_baselink_pose.pose.position.y
                per.status = 4
                person_arr.peopleSet.append(per)

            return person_arr, pose_stamped_first_face
        else:
            rospy.logerr("Message without faces in publisher, shouldn't happen")
            return None, None

    def run(self):
        """Publishing in topics (depending on rate of publication on face detections)"""
        while not rospy.is_shutdown():
            while not self.new_face_msg:
                rospy.sleep(0.1)
            tracker_msg, pointstamped_msg = self.create_tracker_msgs()
            if not tracker_msg: # we got some empty msgs... dont anything then with that msg
                self.new_face_msg = False
                continue
            self.tracker_pub.publish(tracker_msg)
            self.tracked_posestamped_pub.publish(pointstamped_msg)
            self.new_face_msg = False
            rospy.sleep(3)  # publish every 3s


if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.sleep(1)  # Give a second to init_node to do its job
    rospy.loginfo("Initializing follow_me_srv")
    follow_person = TrackFromFaces()
    follow_person.run()
