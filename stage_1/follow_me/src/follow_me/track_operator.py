#! /usr/bin/env python
# vim: expandtab ts=4 sw=4
### FOLOW_OPERATOR.PY ###
"""

@author: Roger Boldu
"""
import rospy
import smach
import math






from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction

from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from util_states.topic_reader import topic_reader
from rospy.core import rospyinfo
from util_states.math_utils import *



MOVE_BASE_TOPIC_GOAL = "/move_by/move_base_simple/goal"
head_client = SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
SECONDS_TO_BE_CONSIDERED_OCCLUDED = 30.0
GO_TO_LOC_TIMEOUT = 0.15 


# i don't like this variables
last_time_occluded = None
not_say_again = False
not_say_again=False
person_height = 1.3 # i't only for looking



        

#TODO: i think that it's not necessary plublish the markers  
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
    # TODO: it's for print a marker.... now it's not necessary
    #marker_state = PublishGeneralMarker(scale=GO_LOC_MARKER_SCALE, marker_name=GO_LOC_MARKER_NAME, marker_type=GO_LOC_MARKER_TYPE, colour=GO_LOC_MARKER_COLOUR)
    #marker_state.execute(class_pose)

    return None


class initTrackOperatorVariables(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['in_personTrackingData'],
                            output_keys=["out_new_tracked_person", "currentFilterTries", "currentUsedFilter"])
    def execute(self, userdata):

        userdata.out_new_tracked_person='22'
        userdata.currentFilterTries='44'
        userdata.currentUsedFilter='33'
        return 'succeeded'
    
    
class resetOccludedTime(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                                 input_keys=['id_personTrackingData'],
                                output_keys=["out_new_tracked_person", "currentFilterTries", "currentUsedFilter"])
        def execute(self, userdata):
                global last_time_occluded #TODO: it's sheet...
                if (last_time_occluded is None):
                    last_time_occluded = rospy.Time.now()
                    last_time_occluded = last_time_occluded.to_sec()
                return 'succeeded'


class FilterAndProcessPeopleTrackerData(smach.State):
    def __init__(self):
    
        smach.State.__init__(self,
                                 outcomes=['succeeded', 'no_plausible_person_found', "tracked_person_is_occluded"],
                                 input_keys=['in_persons_detected', 'out_new_tracked_person', "currentFilterTries", "currentUsedFilter"],
                                 output_keys=['out_new_tracked_person', "currentFilterTries", "currentUsedFilter"])
    def execute(self, userdata):
        rospy.sleep(5)
        rospy.loginfo("i'm in the filter state, now it's a dummy state")
        userdata.out_new_tracked_person='33'
        userdata.currentFilterTries='22'
        userdata.currentUsedFilter='04'
        return 'succeeded'
    
#TODO mo haig de mirar    
class changePersonDataToNotFollow(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded','reduced_distance'],
                                 input_keys=['out_new_tracked_person'],
                                output_keys=["out_new_tracked_person"])
        def execute(self, userdata):
                modified_person = userdata.out_new_tracked_person
                alfa = math.atan2(modified_person.y, modified_person.x)
                s = math.sin(alfa)
                c = math.cos(alfa)
                userdata.out_new_tracked_person='33'

                dist = 0.1
                modified_person.x = c * dist
                modified_person.y = s * dist


                userdata.out_new_tracked_person = modified_person

                #FIXME
                global last_time_occluded # vaia guarrada
                if (last_time_occluded is not None):
                    global not_say_again
                    if (not_say_again):
                        return 'succeeded'
                    current_time = rospy.Time.now()
                    current_time = current_time.to_sec()

                    elapsed_time = current_time - last_time_occluded
                    
                    if (elapsed_time > SECONDS_TO_BE_CONSIDERED_OCCLUDED):
                        #we try to set the parameter till it doesn't throw any error
                        while True:
                            try:
                                #FIXME ROBOCUP HACK
                                #rospy.set_param("/params_learn_and_follow_operator_test/distance_to_human", ELEVATOR_DISTANCE_TO_HUMAN)
                                break
                            except Exception as ex:
                                rospy.loginfo("There was an error while trying to set the parameter. Trying again...")
                        not_say_again=True
                        return "reduced_distance"
    
                    return 'succeeded'

class publishFollowMeTargetId(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                 outcomes=['succeeded'],
                                 input_keys=['out_new_tracked_person'],output_keys=['in_new_tracked_person'])
    def execute(self, userdata):
        #TODO: la intencio es printar tota la info del que estic seguin, aixo i es en els antics i te bona pinta
        # printPersonStatus(userdata.in_new_tracked_person)
        rospy.loginfo(' Following the target Id => ' + str(userdata.out_new_tracked_person.targetId) +
                    ' Status of the target => ' + str(userdata.out_new_tracked_person.targetStatus))
               
        rospy.sleep(5)
        userdata.out_new_tracked_person='22'
        rospy.loginfo("i'm in the filter state, now it's a dummy state")
        return 'succeeded'


class detectElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    def execute(self, userdata):
        #TODO: i will have to lock if i'm realy near of the person, if it's true i will have to pass to the other 
        #part of the test
        rospy.sleep(5)
        rospy.loginfo("i'm looking if i'm in the elevator......")

        return 'aborted'


# it create the goal
class SetNewLocation(smach.State):
    def __init__(self, distanceToHuman=0.9):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['in_new_tracked_person'],
                             output_keys=['out_new_navgoal'])

        while True:
            try:
                
                self._distanceToHuman = rospy.get_param("/params_learn_and_follow_operator_test/distance_to_human", distanceToHuman)
                break
            except Exception as ex:
                rospy.logerr("There was an error while trying to get the parameter. Trying again...")

    def execute(self, userdata):
        rospy.loginfo('### ENTERING "SET_NEW_LOCATION" STATE ###', 3)

        # getting the distance to human from the parameters file that may have changed by a listened command
        # TODO super weird code snippet
        while True:
            try:
                self._distanceToHuman = rospy.get_param("/params_learn_and_follow_operator_test/distance_to_human")
                break
            except Exception as ex:
                rospy.loginfo("There was an error while trying to get the parameter. Trying again...")

        #Calculating vectors for the position indicated
        new_pose = Pose()
        new_pose.position.x = userdata.in_new_tracked_person.x
        new_pose.position.y = userdata.in_new_tracked_person.y
        unit_vector = normalize_vector(new_pose.position)
        position_distance = vector_magnitude(new_pose.position)
        rospy.loginfo(" Position data from Reem to person:")
        rospy.loginfo(" Position vector : " + str(new_pose.position))
        rospy.loginfo(" Unit position vector : " + str(unit_vector))
        rospy.loginfo(" Position vector distance : " + str(position_distance))
        rospy.loginfo(" Distance to human : " + str(self._distanceToHuman))

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
            rospy.loginfo(" Person too close => not moving, just rotate")

        dist_vector = multiply_vector(unit_vector, distance_des)

        alfa_degree = math.degrees(alfa)

        rospy.loginfo(' Final robot movement data:')
        rospy.loginfo(' Distance from robot center to person : ' + str(position_distance))
        rospy.loginfo(' Person and Reem wanted distance (distance to human) : ' + str(self._distanceToHuman))
        rospy.loginfo(' Distance that REEM will move towards the person : ' + str(distance_des))
        rospy.loginfo(' Degrees that REEM will rotate : ' + str(alfa_degree))

        nav_goal = PoseStamped()
        nav_goal.header.stamp = rospy.Time.now()
        nav_goal.header.frame_id = "/base_link"
        nav_goal.pose.position = dist_vector
        nav_goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, alfa))
        userdata.out_new_navgoal = nav_goal
        rospy.loginfo(' This is the Nav Goal We send to REEM: ' + str(nav_goal))

        return 'succeeded'


# it publish the goal
class GoToLocation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'], input_keys=['in_new_navgoal'])
        self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, PoseStamped)

    def execute(self, userdata):
        rospy.loginfo('### ENTERING "GO_TO_LOCATION" STATE ###', 3)
        #TODO: it's used to see in a picture i understand
        # PublishGoToLocationMarkers(userdata.in_new_navgoal.pose)

        rospy.loginfo(' Data that will be sent to the topic ' + MOVE_BASE_TOPIC_GOAL + 
                       ' the pose:\n' + str(userdata.in_new_navgoal))
        self.pub = rospy.Publisher(MOVE_BASE_TOPIC_GOAL, PoseStamped)
        self.pub.publish(userdata.in_new_navgoal)

        rospy.sleep(GO_TO_LOC_TIMEOUT)

        return 'succeeded'


# it look to the person
class lookAtLocation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'], input_keys=['in_new_tracked_person'])

    def execute(self, userdata):                
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

                return 'succeeded'


class topic_reader2(smach.StateMachine):
    def __init__(self,topic_name='/people_tracking/peopleSet', topic_type=Pose(), topic_time_out=90):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'], output_keys=['topic_output_msg'])

    def execute(self, userdata):                
        rospy.loginfo("im in the topic reader fake state...")
        rospy.sleep(5)
        userdata.topic_output_msg='hello'
        
        return 'succeeded'

class TrackOperator(smach.StateMachine):
    def __init__(self, distToHuman=0.9):
        smach.StateMachine.__init__(self,
                                    ['succeeded', 'preempted', 'aborted'],
                                    input_keys=['in_personTrackingData'])
        with self:

            self.in_new_tracked_person='22'
            #TODO: no em cal el remaping
            smach.StateMachine.add(
                                'INIT_TRACK_OPERATOR_VARIABLES',
                                initTrackOperatorVariables(),
                                transitions={'succeeded': 'GRAB_PEOPLE_TRACKER_DATA'},
                                remapping={"in_personTrackingData": "in_personTrackingData",
                                          "out_new_tracked_person": "out_new_tracked_person",
                                          "currentFilterTries": "currentFilterTries",
                                          "currentUsedFilter": "currentUsedFilter"})



            #it will stay in this state since it recive a message
            # TODO: i will like to put none, because i won't to get the message
            # i have create a fake one...
            smach.StateMachine.add('GRAB_PEOPLE_TRACKER_DATA',
                                   topic_reader2(topic_name='/people_tracking/peopleSet', topic_type=Pose(), topic_time_out=90),
                                   transitions={'succeeded': 'FILTER_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                                'preempted': 'preempted',
                                                'aborted': 'GRAB_PEOPLE_TRACKER_DATA'},
                                   remapping={'topic_output_msg':'out_persons_detected'})

#TODO: it's in another file... at the moment is a dummy state
            smach.StateMachine.add('FILTER_AND_PROCESS_PEOPLE_TRACKER_DATA',
                                   FilterAndProcessPeopleTrackerData(),
                                   transitions={'succeeded': 'RESET_OCCLUDED_TIME',
                                                'no_plausible_person_found': 'GRAB_PEOPLE_TRACKER_DATA',
                                                'tracked_person_is_occluded': 'CHANGE_PERSON_DATA_TO_NOT_FOLLOW'},
                                   remapping={'in_persons_detected': 'out_persons_detected',
                                              "out_new_tracked_person":"out_new_tracked_person",
                                              "currentFilterTries": "currentFilterTries",
                                              "currentUsedFilter": "currentUsedFilter"})


            smach.StateMachine.add('RESET_OCCLUDED_TIME',resetOccludedTime(),
                                   transitions={'succeeded': 'PUBLISH_TARGET_ID'})

                

            smach.StateMachine.add('CHANGE_PERSON_DATA_TO_NOT_FOLLOW',
                                   changePersonDataToNotFollow(),
                                   transitions={'succeeded': 'PUBLISH_TARGET_ID',
                                                'reduced_distance': 'PUBLISH_TARGET_ID'},
                                   remapping={"out_new_tracked_person": "out_new_tracked_person"})


# it will publicate de status of the following, the place that the robot is and all the information,
# basicli it's a debug state
            smach.StateMachine.add('PUBLISH_TARGET_ID',
                                   publishFollowMeTargetId,
                                   transitions={'succeeded': 'DETECT_ELEVATOR'},
                                   remapping={"out_new_tracked_person":"out_new_tracked_person"})

            # TODO: this is not a state that it will have to be hear... i don't find the logic..
            smach.StateMachine.add("DETECT_ELEVATOR",
                                   detectElevator(),
                                   transitions={'aborted': "SET_NEW_LOCATION",'succeeded':'succeeded'})

            smach.StateMachine.add('SET_NEW_LOCATION',
                                   SetNewLocation(distToHuman),
                                   transitions={'succeeded': 'GO_TO_LOCATION'},
                                   remapping={'in_new_tracked_person': 'out_new_tracked_person',
                                              'out_new_navgoal': 'out_new_navgoal'})

            smach.StateMachine.add('GO_TO_LOCATION',
                                   GoToLocation(),
                                   transitions={'succeeded': 'LOOK_AT_PERSON', 'preempted':
                                             "LOOK_AT_PERSON", 'aborted': "LOOK_AT_PERSON"},
                                   remapping={'in_new_navgoal': 'out_new_navgoal'})


# TODO: The sensors will have this in mind?
#this move the head to the site that are the person
            smach.StateMachine.add('LOOK_AT_PERSON',
                                   lookAtLocation(), input_keys=["in_new_tracked_person"],
                                   transitions={'succeeded': "GRAB_PEOPLE_TRACKER_DATA",
                                                'preempted': "GRAB_PEOPLE_TRACKER_DATA",
                                                'aborted': "GRAB_PEOPLE_TRACKER_DATA"},
                                   remapping={'in_new_tracked_person': 'out_new_tracked_person'})


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''