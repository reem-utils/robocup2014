#! /usr/bin/env python

"""
@author: Cristina De Saint Germain
8 March 2014
"""

import smach
import rospy
import rosservice
import rosgraph.masterapi
from util_states.colors import Colors

TOPIC_LIST_NAMES = [  # Topics and Actions
                    ##################### Topics #####################
                    "/amcl_pose",
                    "/scan_filtered",    
                    "/asr_event", 
                    "/head_mount_xtion/gestures",
                    "/object_detect/recognize"]

SERVICES_LIST_NAMES = ["/pal_face/recognizer"]  # #################### Services #####################
                       
ACTION_LIST_NAMES = [  # #################### Actions #####################
                    "/play_motion",
                    "/move_base",
                    "/sound"]

PARAMS_LIST_NAMES = [  # #################### Params #####################
                     "/mmap/poi/submap_0/party_room",
                     "/mmap/poi/submap_0/storage_room",
                     "/mmap/poi/submap_0/leave_room",
                     "/mmap/object/information/coke"]
                     

class UserdataHacked():
    def __init__(self):
        self.anything = "test"


class CheckDependencesState(smach.State):
    """CheckDependencesState.

    Use this state to check:
        If all actions and services that your State Machine need are running; if is some node publishing on a specific topic that you need;
        if all locations that you need send the robot can be translated from coord_translator;

    Steps of this State:
        Check if mandatory services/actions/topics are running.
        Check specific topics required by your State Machine are being published.
        Check specific services required by your State Machine.
        Check specific actions required by your State Machine.
        Check if all the locations that you need were set on the map.
        Check if all objects that the robot should recognize (drinks, food, snacks) are at the database, asking the service /object_translator.
        #Send the robot arms to a position out of self colision.

    IMPORTANT:
        To use this State, you should include in your manifest the package 'coord_translator'

    This State Machine check if the topics, actions, services are running and if is possible translate locations in the map.
    """

    def __init__(self, topic_names=TOPIC_LIST_NAMES, service_names=SERVICES_LIST_NAMES, action_names=ACTION_LIST_NAMES, params_names=PARAMS_LIST_NAMES, input_keys=[], output_keys=[]):
        """Constructor for CheckDependencesState

        @type topic_names: list of strings
        @param topic_names: The topic names required by your State Machine.

        @type service_names: list of strings
        @param service_names: The service names required by your State Machine.

        @type action_names: list of strings
        @param action_names: The action names required by your State Machine.

        @type map_locations: list of strings
        @param map_locations: All the locations 'on map' that you need in your State Machine.

        @type object_names: list of strings
        @param object_names: All objects (drinks, foods, snacks) that the robot should recognize by your State Machine.

        """
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])
        if(topic_names is not None and "list" not in str(type(topic_names))):
            raise ValueError("topic_names need be of type 'list' and the type is %s" % type(topic_names))
        if(service_names is not None and "list" not in str(type(service_names))):
            raise ValueError("service_names need be of type 'list' and the type is %s" % type(service_names))
        if(action_names is not None and "list" not in str(type(action_names))):
            raise ValueError("action_names need be of type 'list' and the type is %s" % type(action_names))
        if(params_names is not None and "list" not in str(type(params_names))):
            raise ValueError("params_names need be of type 'list' and the type is %s" % type(params_names))
        

        self.topic_names = topic_names
        self.service_names = service_names
        self.action_names = action_names
        self.params_names = params_names
        self.rostopic = rosgraph.masterapi.Master('/rostopic')
        self.coordinates = None  # Coordinates 'of kitchen for example', in the map.
        self.object_id = None  # Id of the objects 'coke for example' at the database.
        self.ALL_OK = True
        self.colors = Colors()

    def _print_title(self, title):
        l = len(title)
        title += " " if l % 2 else ""
        for i in range((60 - l) / 2):
            title = "-" + title + "-"
        rospy.loginfo(self.colors.BACKGROUND_GREEN + "%s%s" % (title, self.colors.NATIVE_COLOR))

    def _print_info(self, text):
        rospy.loginfo(self.colors.GREEN_BOLD + text + self.colors.NATIVE_COLOR)

    def _print_warning(self, text):
        rospy.logwarn(self.colors.YELLOW_BOLD + text + self.colors.NATIVE_COLOR)

    def _print_fatal(self, text):
        rospy.logfatal(self.colors.RED_BOLD + text + self.colors.NATIVE_COLOR)

    def __check_service(self, service_name):
        if rosservice.get_service_type(service_name):  # None
            self._print_info("Checking service '%s': OK" % service_name)
            return 'succeeded'
        else:
            self.ALL_OK = False
            self._print_fatal("Checking service '%s': Failed" % service_name)
            return 'aborted'

    def __check_topic(self, topic_name):
        publishers = rosgraph.Master('rostopic').getSystemState()[0]
        if any([x for x in publishers if x[0]==topic_name]):
            self._print_info("Checking topic '%s': OK" % topic_name)
        else:
            self.ALL_OK = False
            self._print_fatal("Checking topic '%s': FAILED" % topic_name)

    def __check_action(self, action_name):
        publishers, sub, serv = rosgraph.masterapi.Master("/").getSystemState()
        for publisher in publishers:
            if publisher[0].startswith(action_name) and publisher[0].endswith("status"):
                self._print_info("Checking action '%s': OK" % action_name)
                return 'succeeded'

        self.ALL_OK = False
        self._print_fatal("Checking action '%s': Failed" % action_name)
        return 'aborted'

    def __check_params(self, param_name):        
        params = rospy.get_param_names()
        if any([x for x in params if x==param_name]):  # None
            self._print_info("Checking params '%s': OK" % param_name)
            return 'succeeded'
        else:
            self.ALL_OK = False
            self._print_fatal("Checking params '%s': Failed" % param_name)
            return 'aborted'

    def check_specific_topics(self):
        self._print_title("CHECKING SPECIFIC TOPICS")
        if self.topic_names:
            for topic in self.topic_names:
                self.__check_topic(topic)
        else:
            self._print_warning("Not checking. 'topic_names' is empty")

    def check_specific_services(self):
        self._print_title("CHECKING SPECIFIC SERVICES")
        if self.service_names:
            for service in self.service_names:
                self.__check_service(service)
        else:
            self._print_warning("Not checking. 'service_names' is empty")

    def check_specific_actions(self):
        self._print_title("CHECKING SPECIFIC ACTIONS")

        if self.action_names:
            for action in self.action_names:
                self.__check_action(action)
        else:
            self._print_warning("Not checking. 'action_names' is empty")

    def check_specific_params(self):
        self._print_title("CHECKING SPECIFIC PARAMS")

        if self.params_names:
            for param in self.params_names:
                self.__check_params(param)
        else:
            self._print_warning("Not checking. 'params_names' is empty")

    def execute(self, userdata):  
        self.check_specific_topics()
        self.check_specific_services()
        self.check_specific_actions()
        self.check_specific_params()

        return 'succeeded' if self.ALL_OK else 'aborted'
    
def main():
    rospy.init_node('cocktail_party_cd')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'CheckDepencences',
            CheckDependencesState(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})    

    sm.execute()


if __name__ == '__main__':
    main()