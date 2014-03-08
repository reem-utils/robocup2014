#! /usr/bin/env python

import rospy
import smach
import paramiko


class SSHClient(smach.State):
    """SSHClient State.

    Use this state to execute a remote command using the ssh protocol.
    The output of the remote computer will be printed on the screen (terminal).

    """

    def __init__(self, hostname, username=None, password=None, port=22):
        """Constructor for SSHClient.

        @type hostname: string
        @param hostname: The computer name where the command will be executed.

        @type username: string
        @param username: The username to login on the remote computer. Default: None

        @type password: string
        @param password: The password for the user $username. Default: None

        @type port: integer
        @param port: The port to connect. Default: 22

        """
        smach.State.__init__(self, input_keys=["in_command"], output_keys=[], outcomes=['succeeded', 'aborted'])
        self.hostname = hostname
        self.username = username
        self.password = password
        self.port = port
        self.exit_status = None
        self.out_messages = None
        self.err_messages = None
        self.RED = "\033[00;31m"
        self.NATIVE_COLOR = "\033[m"
        self.BOLD = "\033[01m"
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.connected = True
            self.ssh_client.connect(self.hostname, username=self.username, password=self.password, port=self.port)
        except Exception, e:
            self.connected = False
            rospy.logerr(self.RED + str(e) + self.NATIVE_COLOR)

    def execute(self, userdata):
        if not self.connected:
            return 'aborted'

        rospy.loginfo(self.BOLD + "Executing '%s' in '%s'%s" % (userdata.in_command, self.hostname, self.NATIVE_COLOR))
        channel = self.ssh_client.get_transport().open_session()
        channel.exec_command(userdata.in_command)
        bufsize = -1
        stdin = channel.makefile('wb', bufsize)
        stdout = channel.makefile('rb', bufsize)
        stderr = channel.makefile_stderr('rb', bufsize)
        self.exit_status = channel.recv_exit_status()
        self.out_messages = stdout.readlines()
        self.err_messages = stderr.readlines()
        for out in self.out_messages:
            rospy.loginfo(out.split("\n")[0])  # Removing '\n'
        for err in self.err_messages:
            rospy.logwarn(self.RED + err.split("\n")[0] + self.NATIVE_COLOR)
        return 'succeeded' if self.exit_status is 0 else 'aborted'

    def get_out_messages(self):
        return self.out_messages

    def get_err_messages(self):
        return self.err_messages

    def get_exit_status(self):
        return self.exit_status
