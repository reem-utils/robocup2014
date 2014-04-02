import rospy
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from actionlib_msgs.msg import GoalStatus
from rostopic import get_topic_type
from colors import Colors
import inspect
import traceback
import datetime

# Configurable constants
PERSON_CHANGE_TIME = rospy.get_param('/common_sm/person_change_time', 6)  # seconds
DETECT_PEOPLE_TIMEOUT = rospy.get_param('/common_sm/detect_people_timeout', 2)  # seconds
FACE_RECOGNITION_CONFIDENCE_THRESHOLD = rospy.get_param('/common_sm/min_face_confidence', 80)  # percentage (0..100)
APPROACH_UNKNOWN_PERSON_TIMEOUT = rospy.get_param('/common_sm/approach_unknown_person_timeout', 20)  # seconds
RECOGNIZE_UNKNOWN_PERSON_TIMEOUT = rospy.get_param('/common_sm/approach_unknown_person_timeout', 10)  # seconds

GRASP_ERROR_PARAM_NAME = "grasp_failed"  # Used to set the errors that can occurs when grasping

# Generic state outcome constants
succeeded = 'succeeded'
preempted = 'preempted'
aborted = 'aborted'

# Purpose-specific state outcome constants
unknown_face = 'unknown_face'
existing_name = 'existing_name'
previously_recognized = "previously_recognized"
colors = Colors()

#extra
o1 = 'topic_reader_outcome1'
o2 = 'topic_reader_outcome2'
o3 = 'topic_reader_outcome3'
o4 = 'topic_reader_outcome4'

# General utility functions

def check_topic(userdata, topic_name):
    """ Check if is some node publishing on a topic """
    topic_name = str(topic_name)
    topic_type, real_topic, msg_eval = get_topic_type(topic_name)  # (None, None, None)
    if real_topic:
        rospy.loginfo("Checking topic '%s': OK" % topic_name)
        return succeeded
    rospy.loginfo("Checking topic '%s': FAILED" % topic_name)
    return aborted

def reset_grasp_errors(userdata=None):
    """This function clear the grasp errors that were set on param $GRASP_ERROR_PARAM_NAME """
    try:
        rospy.set_param(GRASP_ERROR_PARAM_NAME, "")
    except Exception as e:
        print e
    return succeeded

def print_grasp_errors(userdata=None):
    """This function print the grasp errors that were set on param $GRASP_ERROR_PARAM_NAME """
    print colors.WHITE_BOLD + "Grasping errors" + colors.NATIVE_COLOR
    errors = rospy.get_param(GRASP_ERROR_PARAM_NAME, '' )
    print colors.RED + errors if len(errors) else colors.GREEN_BOLD + "No grasp errors found"
    print colors.NATIVE_COLOR
    return succeeded


def set_grasp_error(error, status=GoalStatus.ABORTED, limit=2):
    """
    Use this method to set the error that occurs when the robot is grasping.
    Will be added to the error the filename and current line where this method was called.
    A rosparam called $GRASP_ERROR_PARAM_NAME will be set with the error.
    Don't forget to clear the $GRASP_ERROR_PARAM_NAME at the beginning of your State Machine
    Example:
        rospy.set_param(GRASP_ERROR_PARAM_NAME, "")

    """
    if status != GoalStatus.SUCCEEDED:
        rospy.logerr(colors.BACKGROUND_RED + str(error) + colors.NATIVE_COLOR)
        frame = inspect.currentframe()
        last_line = str(traceback.extract_stack(f=frame, limit=limit)[0][:-1])
        info = rospy.get_param(GRASP_ERROR_PARAM_NAME, "")
        now = str(datetime.datetime.now())
        concat = info + str(error) + ":\n" + last_line +  " " + now.rsplit('.', 1)[0] + "\n\n"
        rospy.set_param(GRASP_ERROR_PARAM_NAME, concat)


def get_position_from_param(param, default_value=None):
    """
    Returns the position contained on a parameter of the Param_server that contains the three coordinates (x,y,z) separately as a Point
    :parameter param the name of the parameter that contains the info
    :return the Point with the pos data
    """
    try:
        pos = rospy.get_param(param)
        return Point(pos[0], pos[1], pos[2])
    except KeyError:
        if default_value:
            return default_value


def get_orientation_from_param(param, default_value=None):
    """
    Returns the orientation contained on a parameter of the Param_server that contains the three coordinates (x,y,z,w) separately as a Quaternion
    :parameter param the name of the parameter that contains the info
    :return the Quaternion with the orientation data
    """
    try:
        pos = rospy.get_param(param)
        return Quaternion(pos[0], pos[1], pos[2], pos[3])
    except KeyError:
        if default_value:
            return default_value


def normalize_object_name(name):
    """
    Given a string that mixes capitals and non-capitals, and a hiphen, returns the first part of the string in non-capitals
    """
    return name.lower().split('-')[0]


# Coordinate transformation functions
global _tl
_tl = None


def transform_pose(pose, src_frame, dst_frame, timeout=10):
    """
    Transforms the given pose from src_frame to dst_frame.
    :param src_frame
    :param dst_frame
    :param timeout the amount of time allowed (in secs) for a transformation (default 3)
    """

    if str(type(pose)) != str(type(Pose())):
        rospy.logwarn(colors.BACKGROUND_RED + "The 'pose' should be a Pose object, not '%s'.%s" % (str(type(pose)).split('\'')[1], colors.NATIVE_COLOR))

    from tf.listener import TransformListener
    assert timeout >= 1

    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time()
    pose_stamped.header.frame_id = src_frame
    pose_stamped.pose = pose

    global _tl
    if not _tl:
        _tl = TransformListener()
        rospy.sleep(0.5)
        timeout -= 0.5

    rospy.loginfo("Transforming position from %s to %s coordinates..." % (
        src_frame, dst_frame))

    # If something fails we'll return the original pose (for testing
    # with mocks when tf isn't available)
    result = pose

    try:
        _tl.waitForTransform(
            target_frame=dst_frame, source_frame=src_frame,
            time=rospy.Time(), timeout=rospy.Duration(timeout))
        pose_transf = _tl.transformPose(dst_frame, pose_stamped)
        result = pose_transf.pose
    except Exception as e:
        rospy.logwarn(colors.BACKGROUND_RED + "Warning! Pose transformation failed! %s%s" % (str(e), colors.NATIVE_COLOR))

    return result

# vim: expandtab ts=4 sw=4
