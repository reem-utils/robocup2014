#! /usr/bin/env python

"""
@author: Cristina De Saint Germain
"""


import smach
import rospy
import socket
from smach_ros import ServiceState
from subprocess import Popen, PIPE#! /usr/bin/env python
from util_states.check_using_robot import CheckUsingRobot, ROBOTS_NAME, ROS_MASTER_URI, COMPUTERNAME
from util_states.colors import Colors 
from util_states.run_command_on_robot import RunCommandOnRobot
from util_states.run_command_local import RunCommandLocal
from smach_ros import SimpleActionState
from roslib import packages

ROBOT_COMPUTER_NAME = ROS_MASTER_URI.split("http://")[1].split(":")[0]  # Assuming always http://$COMPUTER:11311
USERNAME = "root"  # SSH login robot
PASSWORD = "palroot"  # SSH password robot

class UserdataHacked():
    def __init__(self):
        self.anything = "test"


class ConnectRobotState(smach.State):
    """ConnectRobotState.

        Add the ip of the local computer on the robot (/etc/hosts file)
        Synchonize the time between the local computer and the robot.
        Display a message warning if the robot is located.
       

    This State Machine check if the topics, actions, services are running and if is possible translate locations in the map.
    """

    def __init__(self, input_keys=[], output_keys=[]):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])
    
        self.ALL_OK = True
        self.colors = Colors()
        self.MY_IPS = self.__get_my_ips()
        self.using_the_robot = CheckUsingRobot(print_checking=False).execute(UserdataHacked()) == 'succeeded'

    def _print_title(self, title):
        l = len(title)
        title += " " if l % 2 else ""
        for i in range((60 - l) / 2):
            title = "-" + title + "-"
        rospy.loginfo(self.colors.BACKGBACKGROUND_GREENROUND_GREEN + "%s%s" % (title, self.colors.NATIVE_COLOR))

    def _print_info(self, text):
        rospy.loginfo(self.colors.GREEN_BOLD + text + self.colors.NATIVE_COLOR)

    def _print_warning(self, text):
        rospy.logwarn(self.colors.YELLOW_BOLD + text + self.colors.NATIVE_COLOR)

    def _print_fatal(self, text):
        rospy.logfatal(self.colors.RED_BOLD + text + self.colors.NATIVE_COLOR)


    def __get_my_ips(self):
        proccess = Popen("ifconfig | grep addr: | cut -d':' -f2 | cut -d' ' -f1", shell=True, stdout=PIPE, stderr=PIPE)
        out, err = proccess.communicate()
        ips = []
        for ip in out.split("\n"):
            if len(ip):
                ips.insert(len(ip), ip)
        return ips

    def add_local_dns_to_robot(self):
        self._print_title("ADDING LOCAL DNS TO ROBOT")
        rospy.loginfo("IPS FOUND:   " + str(self.MY_IPS))

        if not self.using_the_robot:
            self._print_warning("Not checking. ROS_MASTER_URI no contains " + str(ROBOTS_NAME))
            return 'succeeded'

        for robot_name in ROBOTS_NAME:
            if robot_name in ROS_MASTER_URI.lower():

                robot_ip = socket.gethostbyname(ROBOT_COMPUTER_NAME)
                command = "ip route get %s" % str(robot_ip)
                out, err = Popen(command, shell=True, stdout=PIPE, stderr=PIPE).communicate()

                route_to_robot = out.split("\n")[0]
                my_ip = route_to_robot.split(" ")[len(route_to_robot.split(" ")) - 2]
                rospy.loginfo("SELECTED IP: %s" % my_ip)

                command = "addLocalDns -u %s -i %s " % (COMPUTERNAME, my_ip)
                status = RunCommandOnRobot(command).execute(UserdataHacked())

                if status == 'succeeded':
                    self._print_info("Successfully added {%s, %s}" % (COMPUTERNAME, my_ip))
                else:
                    self.ALL_OK = False
                    self._print_fatal("Failed adding {%s, %s}." % (COMPUTERNAME, my_ip))
                return status

#        self.ALL_OK = False
        self._print_warning("Not added. Running on the robot.")

    def synchronize_time(self):
        self._print_title("SYNCHRONIZING TIME")

        if not self.using_the_robot:
            self._print_warning("Not synchronizing. ROS_MASTER_URI no contains %s " % ROBOTS_NAME)
            return 'succeeded'

        for robot_name in ROBOTS_NAME:
            if robot_name in ROS_MASTER_URI.lower():
                command = "ntpdate -u " + str(ROBOT_COMPUTER_NAME)
                if RunCommandLocal(command=command, sudo_enabled=True).execute(UserdataHacked()) == 'succeeded':
                    self._print_info("Synchronizing time: OK")
                else:
                    self.ALL_OK = False
                    self._print_fatal("Synchronizing time: FAILED")
                break


    def execute(self, userdata):
        self.add_local_dns_to_robot()
        self.synchronize_time()

        return 'succeeded' if self.ALL_OK else 'aborted'
