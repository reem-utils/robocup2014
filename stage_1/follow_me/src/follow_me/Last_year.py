#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###

import rospy
import smach
import math

from pal_smach_utils.utils.global_common import succeeded, preempted, aborted, transform_pose
from iri_perception_msgs.msg import peopleTrackingArray, peopleTracking
from pal_smach_utils.utils.topic_reader import TopicReaderState
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from pal_smach_utils.utils.math_utils import vector_magnitude, normalize_vector, multiply_vector, euclidean_distance, euclidean_distance_3d
from pal_smach_utils.utils.publish_marker import PublishGeneralMarker

from smach import CBState
from pal_smach_utils.utils.colors import Colors
colors = Colors()

from pal_smach_utils.speech.sound_action import SpeakActionState

from pal_smach_utils.utils.timeout_container import SleepState

from std_msgs.msg import Int32

from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction

from actionlib import SimpleActionClient

from pal_smach_utils.utils.debug import debugPrint

from pal_smach_utils.utils.gesture_recognition import GestureRecognition
from pal_smach_utils.navigation.move_to_caller import MoveToCallerStateMachine

from smach_ros import SimpleActionState
from rospy.rostime import Duration


MOVE_BASE_TOPIC_GOAL = "/move_by/move_base_simple/goal"

GO_TO_LOC_TIMEOUT = 0.15

#####################################################################
#GoToLOcationMarker
GO_LOC_MARKER_NAME = "/track_operator/nav_goal"
GO_LOC_MARKER_TYPE = "ARROW"
GO_LOC_MARKER_COLOUR = "BLUE"
GO_LOC_MARKER_SCALE = 1.0

#####################################################################

MAX_LEARN_DISTANCE = rospy.get_param('/params_follow_me/max_distance', 2.0)
MAX_LEARN_DISPLACEMENT = rospy.get_param('/params_follow_me/max_displace', 1.0)
MAX_STILL_VELOCITY = rospy.get_param('/params_follow_me/max_still_velocity', 1.0)
MAX_LAST_POSITION_RADIUS = 1.0
MAX_DISTANCE_RADIUS = 10000
MAX_DISTANCE_TO_ROBOT_RADIUS = 2.5

TO_BE_REMOVED_MASK = 0x01
OCCLUDDED_MASK = 0x02
CANDIDATE_MASK = 0x04
LEGGED_TARGET_MASK = 0x08
VISUALLY_CONFIRMED_MASK = 0x10
FRIEND_IN_SIGHT_MASK = 0x20
BACK_LEARNT_MASK = 0x40
FACE_LEARNT_MASK = 0x80

pt_learning_zone_marker_pub = rospy.Publisher("/LEARNING_ZONE_people_tracker_test/", Marker)
learning_zone_marker = Marker()

ELEVATOR_DISTANCE_TO_HUMAN = 0.65

last_time_occluded = None

not_say_again = False

SECONDS_TO_BE_CONSIDERED_OCCLUDED = 30.0

robot_head_heights = [1.9, 1.7, 1.5]
robot_head_heights_index = 0
DETECT_FACE_TIMEOUT = 4.0
head_client = SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)

person_height = 1.3

from pal_vision_msgs.msg import FaceDetection, FaceDetections


def isOccluded(person, peopleSet=None):
    if (person.targetStatus & OCCLUDDED_MASK == OCCLUDDED_MASK):
        return True
    #FIXME
    '''if (peopleSet is not None):
person_pose = Pose()
person_pose.position.x = person.x
person_pose.position.y = person.y
person_distance = vector_magnitude(person_pose.position)
person_alfa = math.atan2(person_pose.position.y, person_pose.position.x)
person_degrees = math.degrees(person_alfa)
for pt_person in peopleSet:
# the person being tracked cannot be occluded by himself!
if (pt_person.targetId != person.targetId):
pt_person_pose = Pose()
pt_person_pose.position.x = pt_person.x
pt_person_pose.position.y = pt_person.y
pt_person_distance = vector_magnitude(pt_person_pose.position)
pt_person_alfa = math.atan2(pt_person_pose.position.y, pt_person_pose.position.x)
pt_person_degrees = math.degrees(pt_person_alfa)
OCCLUSION_DISTANCE_OFFSET = 0.5
OCCLUSION_DEGREES = 5.0

if (pt_person_distance < (person_distance - OCCLUSION_DISTANCE_OFFSET) and math.fabs(pt_person_degrees - person_degrees) < OCCLUSION_DEGREES):
# the person is considered occluded
debugPrint(
colors.BACKGROUND_YELLOW +
"The person being tracked was occluded by this other person: " + str(pt_person) +
colors.NATIVE_COLOR, 0)
debugPrint(
colors.BACKGROUND_YELLOW +
"The data that was considered an occlusion is: " +
colors.NATIVE_COLOR, 0)
debugPrint(
colors.BACKGROUND_YELLOW +
" Distance of person tracked and person occluding: " +
str(pt_person_distance) +
" and " +
str(person_distance) +
" with an offset of " +
str(OCCLUSION_DISTANCE_OFFSET) +
colors.NATIVE_COLOR, 0)
debugPrint(
colors.BACKGROUND_YELLOW +
" Degress between person tracked and person occluding: " +
str(math.fabs(pt_person_degrees - person_degrees)) +
" with a limit of degrees to consider occlusion of " +
str(OCCLUSION_DEGREES) +
colors.NATIVE_COLOR, 0)
return True'''

    return False


def isLeggedTarget(person):
    if (person.targetStatus & LEGGED_TARGET_MASK == LEGGED_TARGET_MASK):
        return True
    return False


def isVisuallyConfirmed(person):
    if (person.targetStatus & VISUALLY_CONFIRMED_MASK == VISUALLY_CONFIRMED_MASK):
        return True
    return False


def isBackLearnt(person):
    if (person.targetStatus & BACK_LEARNT_MASK == BACK_LEARNT_MASK):
        return True
    return False


def isFaceLearnt(person):
    if (person.targetStatus & FACE_LEARNT_MASK == FACE_LEARNT_MASK):
        return True
    return False


def printPersonStatus(person):
    debugPrint(colors.BACKGROUND_YELLOW + "STATUS of the target id => " + str(person.targetStatus) + colors.NATIVE_COLOR, 2)
    if (isOccluded(person)):
        debugPrint(colors.BACKGROUND_BLUE + "OCCLUDED TARGET" + colors.NATIVE_COLOR, 2)
    if (isLeggedTarget(person)):
        debugPrint(colors.GREEN + "LEGGED TARGET" + colors.NATIVE_COLOR, 2)
    if (isVisuallyConfirmed(person)):
        debugPrint(colors.BACKGROUND_GREEN + "VISUALLY CONFIRMED" + colors.NATIVE_COLOR, 2)
    if (isBackLearnt(person)):
        debugPrint(colors.BLUE + "BACK LEARNT" + colors.NATIVE_COLOR, 2)
    if (isFaceLearnt(person)):
        debugPrint(colors.BACKGROUND_BLUE + "FACE LEARNT" + colors.NATIVE_COLOR, 2)


def applyFilterOne(tracked_person=None, peopleSet=None):
    '''
Tries to find the last person we were tracking. To do this we only have to
look for the same target id.
'''
    debugPrint(
        colors.YELLOW +
        ' APPLYING FILTER N. 1: Trying to find the last person tracked with the same id...' +
        colors.NATIVE_COLOR, 2)

    if (tracked_person is not None and peopleSet is not None):

        debugPrint(
            colors.YELLOW +
            " data of the last person tracked\n" +
            str(tracked_person) +
            colors.NATIVE_COLOR, 3)

        targetId = tracked_person.targetId
        for person in peopleSet:
            debugPrint(
                colors.CYAN +
                " data of the person detected: \n" +
                str(person) +
                colors.NATIVE_COLOR, 3)
            if (person.targetId == targetId):
                debugPrint(
                    colors.GREEN +
                    ' [PASS] FILTER N. 1: The same target id was found in the new people set.' +
                    colors.NATIVE_COLOR, 1)
                return person

    debugPrint(
        colors.RED +
        ' [FAIL] FILTER N. 1 : unable to find the last person tracked.' +
        colors.NATIVE_COLOR, 1)

    return None


def applyFilterTwo(tracked_person=None, peopleSet=None):
    '''
Tries to find a person within a circle of MAX_LAST_POSITION_RADIUS radius around the last position
of the last tracked person.
'''
    debugPrint(
        colors.YELLOW +
        ' APPLYING FILTER N. 2: Selecting a person only near the last tracked position...' +
        colors.NATIVE_COLOR, 2)

    if (tracked_person is not None and peopleSet is not None):

        debugPrint(
            colors.YELLOW +
            " data of the last person tracked\n" +
            str(tracked_person) +
            colors.NATIVE_COLOR, 3)

        nearest_person_inside_radius = None
        min_distance = MAX_LAST_POSITION_RADIUS
        old_pose = Pose()
        old_pose.position.x = tracked_person.x
        old_pose.position.y = tracked_person.y
        for person in peopleSet:
            debugPrint(
                colors.CYAN +
                " data of the person detected: \n" +
                str(person) +
                colors.NATIVE_COLOR, 3)
            person_pose = Pose()
            person_pose.position.x = person.x
            person_pose.position.y = person.y
            distance_from_old_pose = euclidean_distance(person_pose, old_pose)
            debugPrint(colors.CYAN + " distance from last position to person: " + str(distance_from_old_pose) + colors.NATIVE_COLOR, 3)

            if (distance_from_old_pose < min_distance):# ''' and isVisuallyConfirmed(person)''' ):
                min_distance = distance_from_old_pose
                nearest_person_inside_radius = person

        if nearest_person_inside_radius is not None:
            debugPrint(
                colors.GREEN +
                ' [PASS] FILTER N. 2: A candidate near the last tracked person was found.' +
                colors.NATIVE_COLOR, 1)
            return nearest_person_inside_radius

    debugPrint(
        colors.RED +
        ' [FAIL] FILTER N. 2 NOT passed: unable to find any candidate near the last tracked position.' +
        colors.NATIVE_COLOR, 1)

    return None


def applyFilterThree(tracked_person=None, peopleSet=None):
    '''
Tries to find the nearest person to the last position
of the last tracked person.
'''
    debugPrint(
        colors.YELLOW +
        ' APPLYING FILTER N. 3: Selecting the nearest visually confirmed person to the last tracked position...' +
        colors.NATIVE_COLOR, 2)

    if (tracked_person is not None and peopleSet is not None):

        debugPrint(
            colors.YELLOW +
            " data of the last person tracked\n" +
            str(tracked_person) +
            colors.NATIVE_COLOR, 3)

        nearest_person = None
        min_distance = MAX_DISTANCE_RADIUS # should be more than enough, since the distance is measured in meters
        old_pose = Pose()
        old_pose.position.x = tracked_person.x
        old_pose.position.y = tracked_person.y
        for person in peopleSet:
            debugPrint(
                colors.CYAN +
                " data of the person detected: \n" +
                str(person) +
                colors.NATIVE_COLOR, 3)
            person_pose = Pose()
            person_pose.position.x = person.x
            person_pose.position.y = person.y
            distance_from_old_pose = euclidean_distance(person_pose, old_pose)
            debugPrint(colors.CYAN + " distance from last position to person: " + str(distance_from_old_pose) + colors.NATIVE_COLOR, 3)

            if (distance_from_old_pose < min_distance and isVisuallyConfirmed(person)):
                min_distance = distance_from_old_pose
                nearest_person = person

        if nearest_person is not None:
            debugPrint(
                colors.GREEN +
                ' [PASS] FILTER N. 3: The nearest candidate to the last tracked person was found.' +
                colors.NATIVE_COLOR, 1)
            return nearest_person

    debugPrint(
        colors.RED +
        ' [FAIL] FILTER N. 3 NOT passed: unable to find any candidate.' +
        colors.NATIVE_COLOR, 1)

    return None


def applyFilterFour(tracked_person=None, peopleSet=None):
    '''
Tries to find the nearest person to the robot.
'''
    debugPrint(
        colors.YELLOW +
        ' APPLYING FILTER N. 4: Selecting the nearest visually confirmed person to the robot...' +
        colors.NATIVE_COLOR, 2)

    if (tracked_person is not None and peopleSet is not None):

        debugPrint(
            colors.YELLOW +
            " data of the last person tracked\n" +
            str(tracked_person) +
            colors.NATIVE_COLOR, 3)

        nearest_person = None
        min_distance = MAX_DISTANCE_TO_ROBOT_RADIUS
        robot_pose = Pose() # the robot position is the origin in the base_link frame
        robot_pose.position.x = 0.0
        robot_pose.position.y = 0.0
        for person in peopleSet:
            debugPrint(
                colors.CYAN +
                " data of the person detected: \n" +
                str(person) +
                colors.NATIVE_COLOR, 3)
            person_pose = Pose()
            person_pose.position.x = person.x
            person_pose.position.y = person.y
            distance_from_robot = euclidean_distance(person_pose, robot_pose)
            debugPrint(colors.CYAN + " distance from tobot to person: " + str(distance_from_robot) + colors.NATIVE_COLOR, 3)

            if (distance_from_robot < min_distance): #''' and isVisuallyConfirmed(person)''' ):
                min_distance = distance_from_robot
                nearest_person = person

        if nearest_person is not None:
            debugPrint(
                colors.GREEN +
                ' [PASS] FILTER N. 4: The nearest candidate to the robot was found.' +
                colors.NATIVE_COLOR, 1)
            return nearest_person

    debugPrint(
        colors.RED +
        ' [FAIL] FILTER N. 4 NOT passed: unable to find any candidate.' +
        colors.NATIVE_COLOR, 1)

    return None


class FilterAndProcessPeopleTrackerData(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=[succeeded, 'no_plausible_person_found', "tracked_person_is_occluded"],
                             input_keys=['in_persons_detected', 'out_new_tracked_person', "currentFilterTries", "currentUsedFilter"],
                             output_keys=['out_new_tracked_person', "currentFilterTries", "currentUsedFilter"])

    def execute(self, userdata):
        debugPrint('### ENTERING "FILTER_AND_PROCESS_PEOPLE_TRACKER_DATA" STATE ###', 3)
        debugPrint(' Number of persons detected %i' % len(userdata.in_persons_detected.peopleSet), 2)

        tracked_person = userdata.out_new_tracked_person
        tries = userdata.currentFilterTries
        usedFilter = userdata.currentUsedFilter

        debugPrint(
            colors.YELLOW_BOLD +
            ' APPLYING FILTERS...' +
            colors.NATIVE_COLOR, 3)

        # FILTER 1
        FILTER_TRIES = 4
        if (usedFilter == 1):
            tries += 1
            if (tries < FILTER_TRIES):
                new_tracked_person = applyFilterOne(tracked_person, userdata.in_persons_detected.peopleSet)
                if (new_tracked_person is not None):
                    userdata.out_new_tracked_person = new_tracked_person
                    userdata.currentFilterTries = 0
                    if (isOccluded(userdata.out_new_tracked_person, userdata.in_persons_detected.peopleSet)):
                        debugPrint(colors.BACKGROUND_BLUE + "Movement and process cancelled till person is not occluded" + colors.NATIVE_COLOR, 2)
                        return "tracked_person_is_occluded"
                    return succeeded
                userdata.currentFilterTries = tries
            else:
                tries = 0
                usedFilter = 2
                userdata.currentUsedFilter = usedFilter

        # FILTER 2
        FILTER_TRIES = 10
        if (usedFilter == 2):
            tries += 1
            if (tries < FILTER_TRIES):
                new_tracked_person = applyFilterTwo(tracked_person, userdata.in_persons_detected.peopleSet)
                if (new_tracked_person is not None):
                    userdata.out_new_tracked_person = new_tracked_person
                    userdata.currentFilterTries = 0
                    userdata.currentUsedFilter = 1
                    if (isOccluded(userdata.out_new_tracked_person, userdata.in_persons_detected.peopleSet)):
                        debugPrint(colors.BACKGROUND_BLUE + "Movement and process cancelled till person is not occluded" + colors.NATIVE_COLOR, 2)
                        return "tracked_person_is_occluded"
                    return succeeded
                userdata.currentFilterTries = tries
            else:
                usedFilter = 3
                userdata.currentUsedFilter = usedFilter

        # FILTER 3 and last filter that will ever execute until success
        if (usedFilter == 3):
            debugPrint(
                colors.RED +
                ' [FAIL]: NO FILTER PASSED: unable to find any plausible candidate. Using last filter till success...' +
                colors.NATIVE_COLOR, 1)
            new_tracked_person = applyFilterFour(tracked_person, userdata.in_persons_detected.peopleSet)
            if (new_tracked_person is not None):
                userdata.out_new_tracked_person = new_tracked_person
                userdata.currentFilterTries = 0
                userdata.currentUsedFilter = 1
                if (isOccluded(userdata.out_new_tracked_person, userdata.in_persons_detected.peopleSet)):
                    debugPrint(colors.BACKGROUND_BLUE + "Movement and process cancelled till person is not occluded" + colors.NATIVE_COLOR, 2)
                    return "tracked_person_is_occluded"
                return succeeded

        debugPrint(
            colors.RED_BOLD +
            ' Last filter tried : ' + str(usedFilter) +
            ' Filter tries : ' + str(tries) +
            colors.NATIVE_COLOR, 2)

        return 'no_plausible_person_found'


class SetNewLocation(smach.State):
    def __init__(self, distanceToHuman=0.9):
        smach.State.__init__(self,
                             outcomes=[succeeded],
                             input_keys=['in_new_tracked_person'],
                             output_keys=['out_new_navgoal'])
        # TODO super weird code snippet
        while True:
            try:
                self._distanceToHuman = rospy.get_param("/params_learn_and_follow_operator_test/distance_to_human", distanceToHuman)
                break
            except Exception as ex:
                debugPrint(str(ex))
                rospy.logerr("There was an error while trying to get the parameter. Trying again...")

    def execute(self, userdata):
        debugPrint('### ENTERING "SET_NEW_LOCATION" STATE ###', 3)

        # getting the distance to human from the parameters file that may have changed by a listened command
        # TODO super weird code snippet
        while True:
            try:
                self._distanceToHuman = rospy.get_param("/params_learn_and_follow_operator_test/distance_to_human")
                break
            except Exception as ex:
                debugPrint(str(ex), 0)
                debugPrint("There was an error while trying to get the parameter. Trying again...", 0)

        #Calculating vectors for the position indicated
        new_pose = Pose()
        new_pose.position.x = userdata.in_new_tracked_person.x
        new_pose.position.y = userdata.in_new_tracked_person.y
        unit_vector = normalize_vector(new_pose.position)
        position_distance = vector_magnitude(new_pose.position)
        debugPrint(" Position data from Reem to person:", 3)
        debugPrint(" Position vector : " + str(new_pose.position), 3)
        debugPrint(" Unit position vector : " + str(unit_vector), 3)
        debugPrint(" Position vector distance : " + str(position_distance), 3)
        debugPrint(" Distance to human : " + str(self._distanceToHuman), 3)

        """
If person is closer than the distance given, we wont move but we might rotate.
We want that if the person comes closer, the robot stays in the place.
Thats why we make desired distance zero if person too close.
"""

        #atan2 will return a value inside (-Pi, +Pi) so we can compute the correct quadrant
        alfa = math.atan2(new_pose.position.y, new_pose.position.x)
        distance_des = 0.0
        if position_distance >= self._distanceToHuman + 0.25: # offset of 0.25 so the robot moves at least 0.25m
            distance_des = position_distance - self._distanceToHuman
        else:
            debugPrint(" Person too close => not moving, just rotate", 3)

        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

        debugPrint(' Final robot movement data:')
        debugPrint(' Distance from robot center to person : ' + str(position_distance), 3)
        debugPrint(' Person and Reem wanted distance (distance to human) : ' + str(self._distanceToHuman), 3)
        debugPrint(' Distance that REEM will move towards the person : ' + str(distance_des), 3)
        debugPrint(' Degrees that REEM will rotate : ' + str(alfa_degree), 3)

        nav_goal = PoseStamped()
        nav_goal.header.stamp = rospy.Time.now()
        nav_goal.header.frame_id = "/base_link"
        nav_goal.pose.position = dist_vector
        nav_goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, alfa))
        userdata.out_new_navgoal = nav_goal
        debugPrint(' This is the Nav Goal We send to REEM: ' + str(nav_goal), 3)

        return succeeded


class UserdataClassForPublishGeneralMarker():
    def __init__(self):
        self.setVar()

    def setVar(self):
        self.in_pose = Pose()


def PublishGoToLocationMarkers(pose):
    """
The input is a Pose(). Publishes a blue sphere in the position of the goal.
"""

    class_pose = UserdataClassForPublishGeneralMarker()
    class_pose.in_pose = pose
    marker_state = PublishGeneralMarker(scale=GO_LOC_MARKER_SCALE, marker_name=GO_LOC_MARKER_NAME, marker_type=GO_LOC_MARKER_TYPE, colour=GO_LOC_MARKER_COLOUR)
    marker_state.execute(class_pose)

    return None


class GoToLocation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, [succeeded, preempted, aborted], input_keys=['in_new_navgoal'])
        self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, PoseStamped)

    def execute(self, userdata):
        debugPrint('### ENTERING "GO_TO_LOCATION" STATE ###', 3)

        PublishGoToLocationMarkers(userdata.in_new_navgoal.pose)

        debugPrint(' Data that will be sent to the topic ' + MOVE_BASE_TOPIC_GOAL + ' the pose:\n' + str(userdata.in_new_navgoal), 3)
        self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, PoseStamped)
        self.pub.publish(userdata.in_new_navgoal)

        rospy.sleep(GO_TO_LOC_TIMEOUT)

        return succeeded


class TrackOperator(smach.StateMachine):
    def __init__(self, distToHuman=0.9):
        smach.StateMachine.__init__(self,
                                    [succeeded, preempted, aborted],
                                    input_keys=['in_personTrackingData'])
        with self:

            @smach.cb_interface(outcomes=[succeeded])
            def initTrackOperatorVariables(userdata):
                userdata.out_new_tracked_person = userdata.in_personTrackingData
                userdata.currentFilterTries = 0
                userdata.currentUsedFilter = 1
                global last_time_occluded
                last_time_occluded = None
                global not_say_again
                not_say_again = False
                return succeeded

            smach.StateMachine.add('INIT_TRACK_OPERATOR_VARIABLES',
                                   CBState(initTrackOperatorVariables,
                                           input_keys=['in_personTrackingData'],
                                           output_keys=["out_new_tracked_person", "currentFilterTries", "currentUsedFilter"]),
                                   transitions={succeeded: 'GRAB_PEOPLE_TRACKER_DATA'},
                                   remapping={"in_personTrackingData": "in_personTrackingData",
                                              "out_new_tracked_person": "out_new_tracked_person",
                                              "currentFilterTries": "currentFilterTries",
                                              "currentUsedFilter": "currentUsedFilter"})

            # When timeout is set to None, TopicReaderState will wait till it reads some data
            # We rely in the people tracker in this case because this state will always finish
            # since the people tracker will send at least an empty array of persons detected
            smach.StateMachine.add('GRAB_PEOPLE_TRACKER_DATA',
                                   TopicReaderState(topic_name='/iri_people_tracking_rai/peopleSet', msg_type=peopleTrackingArray, timeout=None),
                                   transitions={succeeded: 'FILTER_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                                preempted: preempted,
                                                aborted: 'GRAB_PEOPLE_TRACKER_DATA'},
                                   remapping={'message': 'out_persons_detected'})

            smach.StateMachine.add('FILTER_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                   FilterAndProcessPeopleTrackerData(),
                                   transitions={succeeded: "RESET_OCCLUDED_TIME",
                                                'no_plausible_person_found': 'GRAB_PEOPLE_TRACKER_DATA',
                                                'tracked_person_is_occluded': 'CHANGE_PERSON_DATA_TO_NOT_FOLLOW'},
                                   remapping={'in_persons_detected': 'out_persons_detected',
                                              'out_new_tracked_person': 'out_new_tracked_person',
                                              "currentFilterTries": "currentFilterTries",
                                              "currentUsedFilter": "currentUsedFilter"})

            @smach.cb_interface(outcomes=[succeeded])
            def resetOccludedTime(userdata):
                global last_time_occluded
                if (last_time_occluded is None):
                    last_time_occluded = rospy.Time.now()
                    last_time_occluded = last_time_occluded.to_sec()
                return succeeded

            smach.StateMachine.add('RESET_OCCLUDED_TIME',
                                   CBState(resetOccludedTime),
                                   transitions={succeeded: 'PUBLISH_TARGET_ID'})

            @smach.cb_interface(outcomes=[succeeded, "reduced_distance"])
            def changePersonDataToNotFollow(userdata):
                modified_person = userdata.out_new_tracked_person

                alfa = math.atan2(modified_person.y, modified_person.x)
                s = math.sin(alfa)
                c = math.cos(alfa)

                dist = 0.1
                modified_person.x = c * dist
                modified_person.y = s * dist

                debugPrint("New data of the person(alfa = " + str(alfa) + "):\n" + str(modified_person), 3)

                userdata.out_new_tracked_person = modified_person

                #FIXME
                global last_time_occluded
                if (last_time_occluded is not None):
                    global not_say_again
                    if (not_say_again):
                        return succeeded
                    current_time = rospy.Time.now()
                    current_time = current_time.to_sec()

                    elapsed_time = current_time - last_time_occluded
                    debugPrint(
                        colors.BACKGROUND_YELLOW +
                        "Elapsed time since last occlusion: " + str(elapsed_time) +
                        colors.NATIVE_COLOR, 0)
                    if (elapsed_time > SECONDS_TO_BE_CONSIDERED_OCCLUDED):
                        #we try to set the parameter till it doesn't throw any error
                        while True:
                            try:
                                #FIXME ROBOCUP HACK
                                #rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", ELEVATOR_DISTANCE_TO_HUMAN)
                                break
                            except Exception as ex:
                                debugPrint(str(ex), 0)
                                debugPrint("There was an error while trying to set the parameter. Trying again...", 0)
                        not_say_again = True
                        return "reduced_distance"

                return succeeded

            smach.StateMachine.add('CHANGE_PERSON_DATA_TO_NOT_FOLLOW',
                                   CBState(changePersonDataToNotFollow, input_keys=['out_new_tracked_person'], output_keys=['out_new_tracked_person']),
                                   transitions={succeeded: 'PUBLISH_TARGET_ID',
                                                "reduced_distance": "PUBLISH_TARGET_ID"},
                                   remapping={'out_new_tracked_person': 'out_new_tracked_person'})

            reduced_distance_text = "I have reduced the distance."
            smach.StateMachine.add('SAY_REDUCED_DISTANCE',
                                   SpeakActionState(reduced_distance_text),
                                   transitions={succeeded: 'PUBLISH_TARGET_ID'})

            @smach.cb_interface(outcomes=[succeeded])
            def publishFollowMeTargetId(userdata):
                printPersonStatus(userdata.in_new_tracked_person)
                #pub = rospy.Publisher("followMe/targetId", Int32)
                #pub.publish(userdata.in_new_tracked_person.targetId)
                debugPrint(
                    colors.BACKGROUND_GREEN +
                    ' Following the target Id => ' + str(userdata.in_new_tracked_person.targetId) +
                    ' Status of the target => ' + str(userdata.in_new_tracked_person.targetStatus) +
                    colors.NATIVE_COLOR, 1)
                return succeeded

            smach.StateMachine.add('PUBLISH_TARGET_ID',
                                   CBState(publishFollowMeTargetId, input_keys=['in_new_tracked_person']),
                                   transitions={succeeded: 'DETECT_ELEVATOR'},
                                   remapping={'in_new_tracked_person': 'out_new_tracked_person'})

            @smach.cb_interface(outcomes=[succeeded])
            def detectElevator(userdata):

                #FIXME will not go inside the if ever, was just an idea for robocup
                person_inside_elevator = False

                if (person_inside_elevator):
                    debugPrint(
                        colors.BACKGROUND_YELLOW +
                        "Person inside elevator! Reducing following distance..." +
                        colors.NATIVE_COLOR, 3)

                    #we try to set the parameter till it doesn't throw any error
                    while True:
                        try:
                            rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", ELEVATOR_DISTANCE_TO_HUMAN)
                            break
                        except Exception as ex:
                            debugPrint(str(ex), 0)
                            debugPrint("There was an error while trying to set the parameter. Trying again...", 0)

                    debugPrint(
                        colors.BACKGROUND_YELLOW +
                        "The following distance was reduced to " + str(ELEVATOR_DISTANCE_TO_HUMAN) + " meters" +
                        colors.NATIVE_COLOR, 3)

                return succeeded

            smach.StateMachine.add("DETECT_ELEVATOR",
                                   CBState(detectElevator),
                                   transitions={succeeded: "SET_NEW_LOCATION"})

            smach.StateMachine.add('SET_NEW_LOCATION',
                                   SetNewLocation(distToHuman),
                                   transitions={succeeded: 'GO_TO_LOCATION'},
                                   remapping={'in_new_tracked_person': 'out_new_tracked_person',
                                              'out_new_navgoal': 'out_new_navgoal'})

            smach.StateMachine.add('GO_TO_LOCATION',
                                   GoToLocation(),
                                   transitions={succeeded: 'LOOK_AT_PERSON', preempted: "LOOK_AT_PERSON", aborted: "LOOK_AT_PERSON"},
                                   remapping={'in_new_navgoal': 'out_new_navgoal'})

            @smach.cb_interface(outcomes=[succeeded, preempted, aborted])
            def lookAtLocation(userdata):
                #FIXME
                head_goal = PointHeadGoal()
                head_goal.target.header.frame_id = "base_link"
                head_goal.target.point.x = userdata.in_new_tracked_person.x
                head_goal.target.point.y = userdata.in_new_tracked_person.y
                head_goal.target.point.z = person_height
                head_goal.pointing_frame = "stereo_link"
                head_goal.pointing_axis.x = 1.0
                head_goal.pointing_axis.y = 0.0
                head_goal.pointing_axis.z = 0.0
                head_goal.max_velocity = 1.5
                head_goal.min_duration.secs = 1.5

                head_client.wait_for_server(rospy.Duration(0.3))

                head_client.send_goal(head_goal)

                return succeeded

            smach.StateMachine.add('LOOK_AT_PERSON',
                                   CBState(lookAtLocation, input_keys=["in_new_tracked_person"]),
                                   transitions={succeeded: "GRAB_PEOPLE_TRACKER_DATA",
                                                preempted: "GRAB_PEOPLE_TRACKER_DATA",
                                                aborted: "GRAB_PEOPLE_TRACKER_DATA"},
                                   remapping={'in_new_tracked_person': 'out_new_tracked_person'})


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


class LearnPerson(smach.StateMachine):
    def __init__(self, distToHuman=0.5):
        smach.StateMachine.__init__(self,
                                    [succeeded, preempted, aborted],
                                    output_keys=['out_personTrackingData'],
                                    input_keys=["in_learn_person"])
        with self:

            @smach.cb_interface(outcomes=["learn", "not_learn"])
            def learnOrNot(userdata):
                if (userdata.in_learn_person):
                    return "learn"
                else:
                    return "not_learn"

            smach.StateMachine.add('LEARN_OR_NOT',
                                   CBState(learnOrNot, input_keys=['in_learn_person']),
                                   remapping={'in_learn_person': "in_learn_person"},
                                   transitions={"learn": 'SAY_STAY_STILL',
                                                "not_learn": "INIT_MOCK_PERSON"})

            intro_text = "Please stay still while I learn how you look like."
            smach.StateMachine.add('SAY_STAY_STILL',
                                   SpeakActionState(intro_text),
                                   transitions={succeeded: 'CREATE_LEARNING_ZONE_MARKER'})

            @smach.cb_interface(outcomes=[succeeded])
            def createLearningZoneMarker(userdata):
                global pt_learning_zone_marker_pub
                global learning_zone_marker
                debugPrint("Creating learning zone marker...", 2)
                pt_learning_zone_marker_pub = rospy.Publisher("/LEARNING_ZONE_people_tracker_test/", Marker)

                learning_zone_marker = Marker()
                learning_zone_marker.header.frame_id = "/base_link"
                learning_zone_marker.ns = "learning_zone_ns"
                learning_zone_marker.id = 0
                learning_zone_marker.type = learning_zone_marker.LINE_STRIP
                learning_zone_marker.action = learning_zone_marker.ADD
                learning_zone_marker.points = [Point(), Point(), Point(), Point()]
                learning_zone_marker.points[0].x = 0.0
                learning_zone_marker.points[0].y = 0.0
                learning_zone_marker.points[0].z = 0.0
                learning_zone_marker.points[1].x = MAX_LEARN_DISTANCE
                learning_zone_marker.points[1].y = MAX_LEARN_DISPLACEMENT
                learning_zone_marker.points[1].z = 0.0
                learning_zone_marker.points[2].x = MAX_LEARN_DISTANCE
                learning_zone_marker.points[2].y = -MAX_LEARN_DISPLACEMENT
                learning_zone_marker.points[2].z = 0.0
                learning_zone_marker.points[3].x = 0.0
                learning_zone_marker.points[3].y = 0.0
                learning_zone_marker.points[3].z = 0.0
                learning_zone_marker.pose.position.x = 0.0
                learning_zone_marker.pose.position.y = 0.0
                learning_zone_marker.pose.position.z = 0.0
                learning_zone_marker.scale.x = 0.1
                learning_zone_marker.color.a = 1.0
                learning_zone_marker.color.r = 1.0
                learning_zone_marker.color.g = 0.3
                learning_zone_marker.color.b = 0.2
                learning_zone_marker.lifetime = rospy.Duration(5.0)
                learning_zone_marker.header.stamp = rospy.Time.now()

                return succeeded

            smach.StateMachine.add('CREATE_LEARNING_ZONE_MARKER',
                                   CBState(createLearningZoneMarker),
                                   transitions={succeeded: 'PUBLISH_LEARNING_ZONE_MARKER'})

            @smach.cb_interface(outcomes=[succeeded])
            def publishLearningZoneMarker(userdata):
                global pt_learning_zone_marker_pub
                global learning_zone_marker
                learning_zone_marker.header.stamp = rospy.Time.now()

                debugPrint("Publishing learning zone marker...", 2)
                pt_learning_zone_marker_pub.publish(learning_zone_marker)

                return succeeded

            smach.StateMachine.add('PUBLISH_LEARNING_ZONE_MARKER',
                                   CBState(publishLearningZoneMarker),
                                   transitions={succeeded: 'LEARN_AND_GRAB_PEOPLE_TRACKER_DATA'})

            # When timeout is set to None, TopicReaderState will wait till it reads some data
            # We rely in the people tracker in this case because this state will always finish
            # since the people tracker will send at least an empty array of persons detected
            smach.StateMachine.add('LEARN_AND_GRAB_PEOPLE_TRACKER_DATA',
                                   TopicReaderState(topic_name='/iri_people_tracking_rai/peopleSet', msg_type=peopleTrackingArray, timeout=5),
                                   transitions={succeeded: 'LEARN_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                                preempted: preempted,
                                                aborted: 'PUBLISH_LEARNING_ZONE_MARKER'},
                                   remapping={'message': 'out_persons_detected'})

            @smach.cb_interface(outcomes=[succeeded, 'anyone_in_front'])
            def showAndProcessPeopleTrackerData(userdata):
                debugPrint(colors.YELLOW + "Showing people tracker data..." + colors.NATIVE_COLOR, 2)

                tracked_person = None
                min_distance = MAX_LEARN_DISTANCE
                for person in userdata.in_persons_detected.peopleSet:
                    debugPrint(colors.CYAN + str(person) + colors.NATIVE_COLOR, 2)
                    displace = abs(person.y)
                    person_pose = Pose()
                    person_pose.position.x = person.x
                    person_pose.position.y = person.y
                    distance = vector_magnitude(person_pose.position)

                    #little hack to use the same variable and function
                    person_pose.position.x = person.vx
                    person_pose.position.y = person.vy
                    velocity = vector_magnitude(person_pose.position)

                    printPersonStatus(person)
                    debugPrint(colors.CYAN + " displace => " + str(displace) + colors.NATIVE_COLOR, 2)
                    debugPrint(colors.CYAN + " distance => " + str(distance) + colors.NATIVE_COLOR, 2)
                    debugPrint(colors.CYAN + " velocity => " + str(velocity) + colors.NATIVE_COLOR, 2)

                    if isVisuallyConfirmed(person) and displace < MAX_LEARN_DISPLACEMENT and distance < min_distance and velocity < MAX_STILL_VELOCITY:
                        debugPrint("CONDITIONS CONDITIONS CONDITIONS", 2)
                        debugPrint("visual consition" + str(isVisuallyConfirmed(person)), 2)
                        debugPrint("displace" + str(displace), 2)
                        debugPrint("distance" + str(distance), 2)
                        debugPrint("velocity" + str(velocity), 2)
                        min_distance = distance
                        tracked_person = person

                if (tracked_person is None):
                    debugPrint(colors.YELLOW_BOLD + " THERE ISN'T ANY STILL CANDIDATE IN FRONT OF THE ROBOT" + colors.NATIVE_COLOR, 1)
                    return 'anyone_in_front'

                userdata.out_personTrackingData = tracked_person
                return succeeded

            smach.StateMachine.add('LEARN_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                   CBState(showAndProcessPeopleTrackerData, input_keys=['in_persons_detected'], output_keys=['out_personTrackingData']),
                                   remapping={'in_persons_detected': 'out_persons_detected',
                                              'out_personTrackingData': 'out_personTrackingData'},
                                   transitions={succeeded: 'SAY_LOOK',
                                                'anyone_in_front': 'PUBLISH_LEARNING_ZONE_MARKER'})

            look_text = "Please, stay still and stand up straight while looking at my head."
            smach.StateMachine.add('SAY_LOOK',
                                   SpeakActionState(look_text),
                                   transitions={succeeded: 'MOVE_HEAD_NEXT_HEIGHT'})

            @smach.cb_interface(outcomes=[succeeded, aborted])
            def moveHeadNextHeight(userdata):

                global robot_head_heights_index
                if (robot_head_heights_index == len(robot_head_heights)):
                    robot_head_heights_index = 0
                    return aborted
                head_z = robot_head_heights[robot_head_heights_index]

                head_goal = PointHeadGoal()
                head_goal.target.header.frame_id = "base_link"
                head_goal.target.point.x = 1.0
                head_goal.target.point.y = 0.0
                head_goal.target.point.z = head_z
                head_goal.pointing_frame = "stereo_link"
                head_goal.pointing_axis.x = 1.0
                head_goal.pointing_axis.y = 0.0
                head_goal.pointing_axis.z = 0.0
                head_goal.max_velocity = 1.5
                head_goal.min_duration.secs = 1.5

                head_client.wait_for_server(rospy.Duration(0.3))

                head_client.send_goal(head_goal)

                rospy.sleep(2.0)

                robot_head_heights_index += 1

                return succeeded

            smach.StateMachine.add('MOVE_HEAD_NEXT_HEIGHT',
                                   CBState(moveHeadNextHeight),
                                   transitions={succeeded: "GET_FACES_AND_PERSON_HEIGHT",
                                                aborted: "SAY_LOOK"})

            def getFacesAndPersonHeight(userdata, message):
                min_face_base_distance = MAX_LEARN_DISTANCE
                height_of_min_face = None
                faces = message.faces

                for face in faces:
                    # the message is already in 3D
                    face_base_distance = math.sqrt((face.position3D.x**2) + (face.position3D.y**2))

                    if (face_base_distance < min_face_base_distance):
                        min_face_base_distance = face_base_distance
                        height_of_min_face = face.position3D.z

                if (height_of_min_face is not None):
                    global person_height
                    person_height = height_of_min_face
                    debugPrint(
                        colors.BACKGROUND_GREEN +
                        "The height of the person being tracked is " + str(person_height) +
                        colors.NATIVE_COLOR, 0)
                    return succeeded
                else:
                    return aborted

            smach.StateMachine.add(
                'GET_FACES_AND_PERSON_HEIGHT',
                TopicReaderState(
                    topic_name='/person/faceDetections',
                    msg_type=FaceDetections,
                    callback=getFacesAndPersonHeight,
                    timeout=DETECT_FACE_TIMEOUT),
                transitions={succeeded: 'FIX_HEAD',
                             aborted: "MOVE_HEAD_NEXT_HEIGHT"})

            @smach.cb_interface(outcomes=[succeeded, preempted, aborted])
            def fixHeadPosition(userdata):
                head_goal = PointHeadGoal()
                head_goal.target.header.frame_id = "base_link"
                head_goal.target.point.x = 1.0
                head_goal.target.point.y = 0.0
                head_goal.target.point.z = person_height
                head_goal.pointing_frame = "stereo_link"
                head_goal.pointing_axis.x = 1.0
                head_goal.pointing_axis.y = 0.0
                head_goal.pointing_axis.z = 0.0
                head_goal.max_velocity = 1.5
                head_goal.min_duration.secs = 1.5

                head_client.wait_for_server(rospy.Duration(5.0))

                head_client.send_goal(head_goal)

                rospy.sleep(2.0)

                return succeeded

            smach.StateMachine.add('FIX_HEAD',
                                   CBState(fixHeadPosition),
                                   transitions={succeeded: "SAY_TURN_AROUND",
                                                preempted: "SAY_TURN_AROUND",
                                                aborted: "SAY_TURN_AROUND"})

            turn_around_text = "Ok! I have seen your face. Now please, turn around."
            smach.StateMachine.add('SAY_TURN_AROUND',
                                   SpeakActionState(turn_around_text),
                                   transitions={succeeded: 'SLEEP_TILL_PERSON_TURNS'})

            smach.StateMachine.add('SLEEP_TILL_PERSON_TURNS',
                                   SleepState(1.0),
                                   transitions={succeeded: 'PUBLISH_TARGET_ID',
                                                preempted: 'PUBLISH_TARGET_ID'})

            @smach.cb_interface(outcomes=[succeeded])
            def publishFollowMeTargetId(userdata):
                #pub = rospy.Publisher("followMe/targetId", Int32)
                #pub.publish(userdata.in_personTrackingData.targetId)
                debugPrint(
                    colors.BACKGROUND_GREEN +
                    ' Following the target Id => ' + str(userdata.in_personTrackingData.targetId) +
                    ' Status of the target => ' + str(userdata.in_personTrackingData.targetStatus) +
                    colors.NATIVE_COLOR, 1)
                return succeeded

            smach.StateMachine.add('PUBLISH_TARGET_ID',
                                   CBState(publishFollowMeTargetId, input_keys=['in_personTrackingData']),
                                   transitions={succeeded: 'SLEEP_TILL_READY_TO_FOLLOW'},
                                   remapping={'in_personTrackingData': 'out_personTrackingData'})

            smach.StateMachine.add('SLEEP_TILL_READY_TO_FOLLOW',
                                   SleepState(2.0),
                                   transitions={succeeded: 'SAY_READY_TO_FOLLOW',
                                                preempted: 'SAY_READY_TO_FOLLOW'})

            ready_text = "I'm ready to follow you."
            smach.StateMachine.add('SAY_READY_TO_FOLLOW',
                                   SpeakActionState(ready_text),
                                   transitions={succeeded: succeeded})

            ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

            '''smach.StateMachine.add("DETECT_PERSON_WAVING",
GestureRecognition(),
transitions={succeeded: "SAY_CAN_SEE_YOU_WAVING",
preempted: "SAY_CANNOT_SEE_YOU_WAVING",
aborted: "SAY_CANNOT_SEE_YOU_WAVING"},
remapping={"out_waving_person_position": "out_person_position"})'''

            not_waving_text = "Sorry, i can't see you. Could you wave?"
            smach.StateMachine.add('SAY_CANNOT_SEE_YOU_WAVING',
                                   SpeakActionState(not_waving_text),
                                   transitions={succeeded: "DETECT_PERSON_WAVING"})

            waving_text = "I can see you. Please, wait me!"
            smach.StateMachine.add('SAY_CAN_SEE_YOU_WAVING',
                                   SpeakActionState(waving_text),
                                   transitions={succeeded: "INIT_WAVE_PERSON"})

            smach.StateMachine.add("DETECT_PERSON_WAVING",
                                   MoveToCallerStateMachine(),
                                   transitions={succeeded: "SAY_CAN_SEE_YOU_WAVING",
                                                preempted: "SAY_CANNOT_SEE_YOU_WAVING",
                                                aborted: "SAY_CANNOT_SEE_YOU_WAVING"},
                                   remapping={"out_waving_person_position": "out_caller_position"})

            @smach.cb_interface(outcomes=[succeeded])
            def initWavePerson(userdata):
                debugPrint(
                    colors.BACKGROUND_BLUE +
                    "Position of the waving person" + str(userdata.in_waving_person_position) +
                    colors.NATIVE_COLOR, 3)
                person = peopleTracking()
                person.targetId = -1
                person.targetStatus = 0
                waving_pose = transform_pose(userdata.in_waving_person_position.pose, "/map", "/base_link")
                person.x = waving_pose.position.x
                person.y = waving_pose.position.y
                person.vx = 0.0
                person.vy = 0.0
                userdata.out_personTrackingData = person
                return succeeded

            smach.StateMachine.add('INIT_WAVE_PERSON',
                                   CBState(initWavePerson, input_keys=["in_waving_person_position"], output_keys=['out_personTrackingData']),
                                   remapping={"in_waving_person_position": "out_waving_person_position",
                                              'out_personTrackingData': 'out_personTrackingData'},
                                   transitions={succeeded: 'SAY_READY_TO_FOLLOW_AGAIN'})

            @smach.cb_interface(outcomes=[succeeded])
            def initMockPerson(userdata):
                person = peopleTracking()
                person.targetId = -1
                person.targetStatus = 0
                person.x = 1.0
                person.y = 0.0
                person.vx = 0.0
                person.vy = 0.0
                userdata.out_personTrackingData = person
                return succeeded

            smach.StateMachine.add('INIT_MOCK_PERSON',
                                   CBState(initMockPerson, output_keys=['out_personTrackingData']),
                                   remapping={'out_personTrackingData': 'out_personTrackingData'},
                                   transitions={succeeded: 'SAY_READY_TO_FOLLOW_AGAIN'})

            ready_text = "Ok! I am following you again."
            smach.StateMachine.add('SAY_READY_TO_FOLLOW_AGAIN',
                                   SpeakActionState(ready_text),
                                   transitions={succeeded: succeeded})