
import smach
import socket
import rospy
import os

COMPUTERNAME = socket.gethostname()
ROS_MASTER_URI = os.environ["ROS_MASTER_URI"]
ROBOTS_NAME = ["reem", "rh2c"]

#ROS_PACKAGE_PATH="/mnt_flash/stacks:/opt/ros/fuerte/share:/opt/ros/fuerte/stacks:/mnt_flash/stacks:/mnt_flash/robocup2013"


class CheckUsingRobot(smach.State):
    """
    This State check if the robot is being used.

    Will check if ROS_MASTER_URI is pointing to the robot or if the COMPUTERNAME contains the robot names.
    """
    def __init__(self, input_keys=[], output_keys=[], print_checking=True, check_computername=True, check_ros_master_uri=True):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])
        self.using_the_robot = False
        self.print_checking = print_checking
        self.out_robot_name = None
        self.check_computername = check_computername
        self.check_ros_master_uri = check_ros_master_uri

    def execute(self, userdata):
        if self.print_checking:
            rospy.loginfo("Checking if contains some of %s in '%s' or '%s'" % (str(ROBOTS_NAME), str(COMPUTERNAME), str(ROS_MASTER_URI)))
        for robot_name in ROBOTS_NAME:
            if self.check_computername is True:
                if robot_name in COMPUTERNAME.lower():
                    self.using_the_robot = True
                    self.out_robot_name = COMPUTERNAME
                    break
            if self.check_ros_master_uri is True:
                if robot_name in ROS_MASTER_URI.lower():
                    self.using_the_robot = True
                    self.out_robot_name = ROS_MASTER_URI.split("http://")[1].split(":")[0]  # Assuming always http://$COMPUTER:$PORT
                    break
        return 'succeeded' if self.using_the_robot else 'aborted'
