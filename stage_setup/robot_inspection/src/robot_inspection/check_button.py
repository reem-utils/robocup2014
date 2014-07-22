#! /usr/bin/env python
'''
Created on 28/06/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
from emergency_button import emergency_button
from util_states.sleeper import Sleeper
from speech_states.say import text_to_say

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
 
class check_button_pressed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pressed','unpressed', 'succeeded', 'preempted'], 
            input_keys=['pressed', 'press'], 
            output_keys=['pressed', 'press'])

    def execute(self, userdata):
        rospy.logwarn("CACA")
        if self.preempt_requested():
            rospy.logwarn('PREEMPT REQUESTED -- Returning Preempted in check_button state')
            return 'preempted'
        if userdata.pressed == True and not userdata.press:
            userdata.press = True
            return 'pressed'
        if userdata.pressed == False and userdata.press:
            userdata.press = False
            return 'unpressed'

        return 'succeeded'

class CheckButton(smach.StateMachine):
    """
    This state machine do the robot inspection. It waits for the emergency button, 
    first that the button was pressed and then that the button was released.  
    
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'end'],
                                     input_keys=[],
                                     output_keys=[])

        with self:
            self.userdata.pressed = False
            self.userdata.press = False
            self.userdata.tts_time_before_speaking = 0
            self.userdata.tts_text = ""
            self.userdata.tts_lang = ""
            
            # Check if the button is pressed
            smach.StateMachine.add(
                'check_button',
                emergency_button(),
                transitions= {'succeeded':'check_button_pressed', 'aborted':'check_button', 'preempted':'preempted'})
                #Cas 1 - Crec que es perdra tota l'estona si esta premut o no
                # transitions= {'succeeded':'check_button_pressed', 'aborted':'aborted', 'preempted':'preempted'})
            
            smach.StateMachine.add(
                'check_button_pressed',
                check_button_pressed(),
                transitions= {'pressed':'say_pressed_button', 'unpressed':'say_not_pressed_button',
                              'succeeded':'sleep_state', 'preempted':'preempted'})
            
            # Sleep
            smach.StateMachine.add(
                'sleep_state',
                Sleeper(2),
                transitions={'succeeded': 'check_button', 'aborted': 'check_button', 'preempted':'preempted'})
            
            # Say button pressed
            smach.StateMachine.add(
                'say_pressed_button',
                text_to_say("Oh you pressed my emergency button, feel free to check me around"),
                transitions= {'succeeded':'check_button', 'aborted':'check_button', 'preempted':'preempted'})

            # Say button is not pressed 
            smach.StateMachine.add(
                'say_not_pressed_button',
                text_to_say("Thank you for unpressing my emergency button. I'll recover my status shortly."),
                transitions= {'succeeded':'end', 'aborted':'end', 'preempted':'preempted'})
            
def main():
    rospy.init_node('robot_inspection_cd')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add(
            'CheckDepencences',
            CheckButton(),
            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})    

    sm.execute()


if __name__ == '__main__':
    main()
