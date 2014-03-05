#! /usr/bin/env python

import smach
from util_states.run_script_on_robot import RunScriptOnRobot


class RunCommandOnRobot(smach.State):
    """RunCommandOnRobot State.

    Use this state to execute a command on the robot.

    If you are running this State from your computer, the robot name will be get from the variable ROS_MASTER_URI.
    The default login and password are defined on the run_script_on_robot.py file.

    If you are running directly on the robot (not from your computer), the command will be executed locally by RunCommandLocal State.

    Be carefull: All commands executed on the robot will run as root user.

    """
    def __init__(self,  command=None, input_keys=[], output_keys=[]):
        """Constructor for RunCommandOnRobot.

        @type command: string
        @param command: The command that you want execute.

        """
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        if command is None:
            raise ValueError("You should set the variable 'command'")

        self.remote_command = RunScriptOnRobot(script_name=command, use_path=False)

    def execute(self, userdata):
        return self.remote_command.execute(userdata)
