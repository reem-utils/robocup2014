#!/usr/bin/env python

import rospy
import smach
import threading

#from global_common import ''preempted'', aborted, preempted, o1, o2, o3, o4


class TopicReaderState(smach.State):
    """
    Custom SMACH state for reading a value from a topic.

    Required parameters:
    @topic_name: the name of the topic to read
    @msg_type: the type of the message

    Optional parameters:
    @callback: a method to call once the first message is received;
               it'll receive (userdata, message) as parameters and
               it must return an outcome or None (for 'succeeded').
    @timeout: a timeout, in seconds, after which this state will
              return with aborted if no message was received.
    @outcomes: possible outcomes (at least 'aborted', ''preempted''
               and an additional outcome).
    @input_keys: input keys
    @output_keys: output keys; ['message'] by default
    @io_keys: io_keys

    If `callback' is None, the first message received will be output
    as user data with key "message".
    """

    _topic_name = None
    _msg_type = None
    _callback = None
    _timeout = None

    _lock = None
    _message = None
    _message_received = None

    def __init__(self, userdata,
            outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['topic_goal'],
            output_keys=['topic_message'], io_keys=[]):
        assert 'aborted' in outcomes and 'preempted' in outcomes
        assert callback is not None or 'message' in output_keys
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self._topic_name = userdata.topic_goal.topic_name
        self._msg_type = userdata.topic_goal.topic_type
        self._callback = callback
        self._timeout = timeout
        self._lock = threading.Lock()

    def _message_handler(self, msg):
        self._lock.acquire()
        if not self._message_received:
            # We just want the first message
            self._message_received = True
            self._message = msg
        self._lock.release()

    def _is_message_received(self):
        self._lock.acquire()
        result = self._message_received
        self._lock.release()
        return result

    def execute(self, userdata):
        rospy.loginfo("Reading topic: " + str(self._topic_name) )
        self._message_received = False
        subscriber = rospy.Subscriber(self._topic_name, self._msg_type,
            callback=self._message_handler, queue_size=1)

        start_time = rospy.Time().now()

        # Wait until we receive a message or it's time to abort
        while not self._is_message_received():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if self._timeout is not None:
                time_running = rospy.Time.now() - start_time
                if time_running > rospy.Duration(self._timeout):
                    return 'aborted'

        # Unsubscribe
        subscriber.unregister()

        # Return the outcome and user data
        if self._callback:
            return self._callback(userdata, self._message) or 'succeeded'
        else:
            userdata.topic_message = self._message
            return 'succeeded'


class TopicReaderStateMultiOutcomes(smach.State):
    """
    Same but with diferent outcome from 'succeeded' 'aborted' and 'preempted'
    It has four outcomes o1,2,3,4. You use the ones you need and the rest
    you can just link the to the same outcome as abort.
    """

    _topic_name = None
    _msg_type = None
    _callback = None
    _timeout = None

    _lock = None
    _message = None
    _message_received = None

    def __init__(self, topic_name, msg_type, callback=None, timeout=1,
            outcomes=['aborted', 'preempted'], input_keys=[],
            output_keys=['message'], io_keys=['general_data']):
        assert 'aborted' in outcomes and 'preempted' in outcomes
        assert callback is None or callable(callback)
        assert callback is not None or 'message' in output_keys
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self._topic_name = topic_name
        self._msg_type = msg_type
        self._callback = callback
        self._timeout = timeout
        self._lock = threading.Lock()

    def _message_handler(self, msg):
        self._lock.acquire()
        if not self._message_received:
            # We just want the first message
            self._message_received = True
            self._message = msg
        self._lock.release()

    def _is_message_received(self):
        self._lock.acquire()
        result = self._message_received
        self._lock.release()
        return result

    def execute(self, userdata):

        self._message_received = False
        subscriber = rospy.Subscriber(self._topic_name, self._msg_type,
            callback=self._message_handler, queue_size=1)

        start_time = rospy.Time().now()

        # Wait until we receive a message or it's time to abort
        while not self._is_message_received():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if self._timeout is not None:
                time_running = rospy.Time.now() - start_time
                if time_running > rospy.Duration(self._timeout):
                    return 'aborted'
            else:
                return 'aborted'

        # Unsubscribe
        subscriber.unregister()

        # Return the outcome and user data
        if self._callback:
            return self._callback(userdata, self._message) or 'succeeded'
        else:
            userdata.message = self._message
            return 'succeeded'


# vim: expandtab ts=4 sw=4
