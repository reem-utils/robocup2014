#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
17 Apr 2014

@author: Cristina De Saint Germain
"""

import rospy
import smach
    
class ConcurrenceRobocup(smach.Concurrence):
    """ConcurrenceRobocup State.

    Use this StateMachine to execute a lot of States or State Machines with concurrence.

    Example of use:
        STATES = [MoveToRoomStateMachine(announcement=None), SpeakActionState("This is a test!")]
        STATE_NAMES = ["MOVE_TO_EXIT", "SPEAbasic_functionalitiesK_SOMETHING"]
        outcome_map = {succeeded: {"MOVE_TO_EXIT": succeeded, "SPEAK_SOMETHING": succeeded}}

        smach.StateMachine.add(
            "MOVE_AND_SPEAK",
            ConcurrenceRobocup(states=STATES, state_names=STATE_NAMES, input_keys=["room_name"], outcome_map=outcome_map)
            )

    You can test it running: roslaunch cocktail_party test_concurrence_robocup.launch

    """
    def __init__(self,
        states=[],
        state_names=[],
        default_outcome='succeeded',
        input_keys=[],
        output_keys=[],
        outcome_map=None,
        outcomes=['succeeded', 'aborted', 'preempted']):
        """Constructor for ConcurrenceRobocup.

        @type states: list of (State|StateMachine)s
        @param states: The list of States that you want execute concurrently, example: states=[SpeakActionState("This is a test")].

        @type state_names: list of strings
        @param state_names: The list of names of your states, example ["SPEAK_SOMETHING"]. If you don't set this variable, the state name will
        be the name of the State. Example: If a State is SpeakActionState then the name will be 1_SpeakActionState.

        @type default_outcome: string
        @param default_outcome:

        @type outcome_map: Dictionary
        @param outcome_map: The dictionary maps the outcome of the concurrence State. If you don't set this variable, and lets suppose that
        the state_names are StateA and StateB, the default outcome_map will be: {succeeded: {'1_A': succeeded, '2_B': succeeded}}

        """

        if str(type(states)) != str(type([])):
            raise ValueError("The variable 'states' should be of type 'list', not %s" % (str(type(states)).split('\'')[1]))
        if str(type(state_names)) != str(type([])):
            raise ValueError("The variable 'state_names' should be of type 'list', not %s" % (str(type(state_names)).split('\'')[1]))

        self.outcome_map = outcome_map
        self.state_names = []

        def __set_state_names():
            if len(state_names) == len(states):
                for state_name in state_names:
                    self.state_names.insert(len(self.state_names), state_name)
            else:
                counter = 0
                for state in states:
                    counter += 1
                    full_state_name = str(type(state)).split('\'')[1]
                    state_name = full_state_name.split('.')[len(full_state_name.split('.')) - 1]
                    state_name = str(counter) + "_" + state_name
                    self.state_names.insert(len(self.state_names), state_name)

        def __set_outcome_map():
            succeeded_map = {}
            if outcome_map is None:
                for state_name in self.state_names:
                    succeeded_map[state_name] = 'succeeded'
                self.outcome_map = {'succeeded': succeeded_map}
            else:
                self.outcome_map = outcome_map

        __set_state_names()
        __set_outcome_map()

        smach.Concurrence.__init__(self,
            outcomes=outcomes,
            default_outcome=default_outcome,
            input_keys=input_keys,
            output_keys=output_keys,
            outcome_map=self.outcome_map)

        with self:
            def __set_states():
                counter = 0
                for state in states:
                    smach.Concurrence.add(
                        self.state_names[counter],
                        state
                        )
                    counter += 1

            __set_states()
