#! /usr/bin/env python

import smach
import rospy
from subprocess import Popen, PIPE
from util_states.colors import Colors


class RunCommandLocal(smach.State):
    """RunCommandLocal State.

    Use this State to run a command on the local computer.

    """
    def __init__(self,  command=None, sudo_enabled=False, input_keys=[], output_keys=[]):
        """Constructor for RunCommandLocal.

        @type command: string
        @param command: The command that you want execute.

        @type sudo_enabled: boolean
        @param sudo_enabled: If true, the command will be executed as sudo. It will open a dialog and you should type your password.

        """
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        if command is None:
            raise ValueError("You should set the variable 'command'")

        self.command = str(command)
        if sudo_enabled is True:
            self.command = "gksudo '" + self.command + "'"

    def execute(self, userdata):
        c = Colors()
        rospy.loginfo(c.WHITE_BOLD + "Running '%s'%s" % (self.command, c.NATIVE_COLOR))
        proccess = Popen(self.command, shell=True, executable="/bin/bash", stdout=PIPE, stderr=PIPE)
        out, err = proccess.communicate()
        for out_msg in out.split('\n'):
            rospy.loginfo(out_msg)

        for err_msg in err.split('\n'):
            rospy.loginfo(c.RED + err_msg + c.NATIVE_COLOR)

        return 'succeeded' if (proccess.returncode == 0) else 'aborted'
