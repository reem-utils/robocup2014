#! /usr/bin/env python
'''
Created on 08/03/2014

@author: Cristina De Saint Germain
@email: crsaintc8@gmail.com

'''
import rospy
import smach
from navigation_states.nav_to_poi import nav_to_poi
from face_states.recognize_face import recognize_face

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

class DummyStateMachine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
            input_keys=[], 
            output_keys=[])

    def execute(self, userdata):
        print "Dummy state just to change to other state"  # Don't use prints, use rospy.logXXXX

        rospy.sleep(3)
        return 'succeeded'


class prepare_poi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'], 
                                input_keys=['num_iterations'],
                                output_keys=['nav_to_poi_name', 'num_iterations'])

    def execute(self, userdata):    
        
        if userdata.num_iterations % 3 == 0:
            userdata.nav_to_poi_name = "point_room_one"
            rospy.loginfo(OKGREEN + "Point_room_one" + ENDC)
        elif userdata.num_iterations % 3 == 1: 
            userdata.nav_to_poi_name = "point_room_two"
            rospy.loginfo(OKGREEN + "Point_room_two" + ENDC)
        else:
            userdata.nav_to_poi_name = "point_room_three"
            rospy.loginfo(OKGREEN + "Point_roomthree" + ENDC)
        
        userdata.num_iterations += 1

        return 'succeeded'
    
class SearchFacesSM(smach.StateMachine):
    """
    The robot goes around a room looking for faces. When it detects the face from
    TC, it stops and return success. 
    
    To do the search we define 3 points
    The robot will go around the 3 points looking for the TC
    When it found it, the state machine return success. Unless it return aborted.

    We need a concurrence state machine:
        -> if face detects the face -> it return success
        -> if walk return success and face aborted -> it return aborted and we need define the new goal
        
    Required parameters:
    No parameters.

    Optional parameters:
    No optional parameters

    No input keys.
    No output keys.
    No io_keys.

    Nothing must be taken into account to use this SM.
    var = cast(raw_input('Sentence:'))
    """
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                     input_keys=['name', 'nav_to_poi_name', 'face'],
                                     output_keys=['face'])

        with self:

            self.userdata.num_iterations = 0
            self.userdata.face = None
            
            # We define the different points
            smach.StateMachine.add(
                'prepare_poi',
                prepare_poi(),
                transitions={'succeeded': 'Concurrence', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            # Concurrence
            sm_conc = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                        default_outcome='succeeded',
                                        input_keys=['name', 'nav_to_poi_name', 'face'],
                                        output_keys=['face'],
                                        outcome_map={'succeeded': {'find_faces': 'succeeded'},
                                                     'aborted': {'walk_to_poi':'succeeded', 
                                                                'find_faces':'aborted'}})
    
            
            with sm_conc:
                # Go around the room 
                smach.Concurrence.add('walk_to_poi', nav_to_poi())                  
          
                # Move head
                smach.Concurrence.add('move_head', DummyStateMachine())
                
                # Search for face
                smach.Concurrence.add('find_faces', recognize_face())

            
            smach.StateMachine.add('Concurrence', sm_conc, 
                                transitions={'succeeded':'succeeded', 'aborted':'prepare_poi'})
            
           

