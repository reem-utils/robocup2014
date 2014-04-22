#! /usr/bin/env python
'''
@author: Sam Pfeiffer
'''
# System stuff
import copy

# ROS stuff
import rospy
import tf

# Msgs
from pal_detection_msgs.msg import FaceDetection, FaceDetections
from follow_me.msg import tracker_people, person, personArray
from pal_detection_msgs.msg import FaceDetections, Detections2d, FaceDetection, Detection2d
from geometry_msgs.msg import PoseStamped, Pose, Point
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
TRACKED_POSESTAMPED_TOPIC = '/move_by/follow'


class TrackFromFaces():
    """
    Subscribe to face detections
    and publish a personArray and also
    a PointStamped with the 3D pose of the person.
    """
    def face_cb(self, data):
        self.last_faces_msg = data
        if len(self.last_faces_msg.faces) > 0:
            self.new_face_msg = True

    def create_tracker_msgs(self):
        current_msg = copy.deepcopy(self.last_faces_msg)
        per = person()  # person msg
        person_arr = personArray()
        pose_stamped_first_face = PoseStamped()
        #rospy.loginfo("waiting for transform...")
        latest_common_tf_time = self.tf_listener.getLatestCommonTime("odom", current_msg.header.frame_id)
        #self.tf_listener.waitForTransform("odom", current_msg.header.frame_id, latest_common_tf_time, rospy.Duration(1.0)) # Not needed
        #rospy.loginfo("got a transform!")

        if current_msg.faces: # should always be filled, but just in case
            target_id = 1
            for face in current_msg.faces:
                # Temporal PoseStamped for transforming into odom frame the 3d pose of the face detection
                face_ps = PoseStamped()
                face_ps.header.frame_id = current_msg.header.frame_id
                face_ps.header.stamp = latest_common_tf_time #rospy.Time.now() #latest_common_tf_time
                face_ps.pose.position = face.position
                face_ps.pose.orientation.w = 1.0

                # transform he pose
                #rospy.loginfo("    Transforming...")
                transform_ok = False
                while not transform_ok: # this is ugly as is polling a lot to TF... but works
                    try:
                        transformed_pose = self.tf_listener.transformPose("odom", face_ps)
                        transform_ok = True
                    except tf.ExtrapolationException:
                        rospy.logwarn("Exception on transforming... trying again.")
                        face_ps.header.stamp = rospy.Time.now()
                #rospy.loginfo("    Transformed!")
                pose_stamped_first_face = transformed_pose
                per.targetId = target_id
                target_id += 1
                per.x = transformed_pose.pose.position.x
                per.y = transformed_pose.pose.position.y
                per.status = 4
                person_arr.peopleSet.append(per)

            return person_arr, pose_stamped_first_face
        else:
            rospy.logerr("Message without faces in publisher, shouldnt happen")
            return None, None
        
        
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

    def run(self):
        """Publishing in topics (depending on rate of publication on face detections)"""
        while not rospy.is_shutdown():
            while not self.new_face_msg:
                rospy.sleep(0.1)
            tracker_msg, pointstamped_msg = self.create_tracker_msgs()
            if not tracker_msg: # we got some empty msgs... dont anything then with that msg
                self.new_face_msg = False
                continue
            self.tracker_pub.publish( tracker_msg )
            self.tracked_posestamped_pub.publish( pointstamped_msg )
            self.new_face_msg = False
            rospy.sleep(3)  # publish every 3s


if __name__ == '__main__':
    rospy.init_node('follow_me_srv')
    rospy.sleep(1)  # Give a second to init_node to do its job
    rospy.loginfo("Initializing follow_me_srv")
    follow_person = TrackFromFaces()
    follow_person.run()
