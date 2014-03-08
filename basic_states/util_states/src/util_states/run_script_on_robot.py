#! /usr/bin/env python

import smach
import rospy
from util_states.ssh_client import SSHClient
from util_states.check_using_robot import CheckUsingRobot, ROBOTS_NAME
from util_states.run_command_local import RunCommandLocal

ROBOT_SCRIPTS_PATH = "/mnt_flash/bin/launch/"  # FIXME: Is it the same location in reemh2 and reemh3?
ROBOT_SCRIPTS_PATH_ROBOCUP = "/mnt_flash/stacks/robocup_stacks/scripts/"
USERNAME = "root"  # SSH login robot
PASSWORD = "r"  # SSH password robot


class RunScriptOnRobot(smach.State):
    """RunScriptOnRobot State.

    Use this state to execute a script on the robot.

    If you are running this State from your computer, the robot name will be get from the variable ROS_MASTER_URI.
    The default login and password are defined on the variables USERNAME and PASSWORD that can be found in this file.

    If you are running directly on the robot (not from your computer), the command will be executed locally by RunCommandLocal State.

    Be carefull: All commands executed on the robot will run as root user.

    Important:
        If you are running this State from your computer, with ROS_MASTER_URI pointing to the robot, the script will probably not works good.
        The problem is because using ssh, the default executable on the target is '/bin/sh', not '/bin/bash'.
        But if you are running this State directly on the robot, it will works perfectly.

    """
    def __init__(self, script_name=None, use_path=True, robot_scripts_path=ROBOT_SCRIPTS_PATH_ROBOCUP, input_keys=[], output_keys=[]):
        """Constructor for RunScriptOnRobot.

        @type script_name: string
        @param script_name: The script name that you want execute.

        @type use_path: boolean
        @param use_path: If True, the variable robot_scripts_path will be considered. This was to be able to execute single commands, not scripts.

        @type robot_scripts_path: string
        @param robot_scripts_path: The path to your scripts. By default, the path is defined on ROBOT_SCRIPTS_PATH_ROBOCUP variable.

        """
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        if script_name is None:
            raise ValueError("You should set the variable 'script_name'")

        self.YELLOW_BOLD = "\033[01;33m"
        self.NATIVE_COLOR = "\033[m"
        self.robot_scripts_path = robot_scripts_path
        self.script_name = script_name
        self.full_script_path = self.robot_scripts_path + self.script_name
        self.use_path = use_path
        self.check = CheckUsingRobot(print_checking=False)

    def execute(self, userdata):

        robot_scripts_path = self.robot_scripts_path
        script_name = self.script_name
        use_path = self.use_path
        command = (str(robot_scripts_path) + str(script_name)) if use_path else str(script_name)

        #If running ON THE ROBOT
        if CheckUsingRobot(print_checking=False, check_ros_master_uri=False).execute(userdata) == 'succeeded':
            return RunCommandLocal(command=command).execute(userdata)

        using_robot = self.check.execute(userdata) == 'succeeded'
        status = 'aborted'
        if using_robot:
            HOST = self.check.out_robot_name
            ssh_client = SSHClient(HOST, username=USERNAME, password=PASSWORD)

            class UserdataHacked():
                def __init__(self):
                    self.in_command = command

            status = ssh_client.execute(UserdataHacked())
        else:
            status = 'succeeded'
            rospy.logwarn(self.YELLOW_BOLD + "Not executed '%s'. ROS_MASTER_URI nor COMPUTER_NAME contains '%s'%s" % (self.script_name, ROBOTS_NAME, self.NATIVE_COLOR))

        return status  # succeeded/aborted
