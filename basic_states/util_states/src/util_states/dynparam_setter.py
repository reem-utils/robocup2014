#! /usr/bin/env python
'''
Created on 09/07/2014

@author: Sammy Pfeiffer

'''

import rospy
import smach
import smach_ros
from dynamic_reconfigure import client

class DynamicParameterSetter(smach.State):
    """A state to set a dynamic parameters

    
    Input Keys:
        param_server_name: name of the dynamic parameter/dynamic reconfigure server. i.e. /move_base
        parameters_dict: dict('key' : value) with the parameters to set, i.e. {'planner_frequency' : 0.0}
    Output Keys:
        standard_error: String thats shows what fails
    
   """

    def __init__(self, param_server_name = None, parameters_dict = None):
        smach.State.__init__(self, input_keys=['param_server_name', 'parameters_dict'], output_keys=['standard_error'], outcomes=['succeeded'])       
        self.param_server_name = param_server_name
        self.parameters_dict = parameters_dict
        
    def execute(self, userdata):  
        # Check if use parameters or input keys
        ps_name = None
        p_dict = None
        if self.param_server_name == None or self.parameters_dict == None:
            ps_name = userdata.param_server_name
            p_dict = userdata.parameters_dict
        else:
            ps_name = self.param_server_name
            p_dict = self.parameters_dict
        
        rospy.loginfo("Trying to connect a service client to '" + ps_name + "' dynamic reconfigure...")
        dynparamclient = client.Client(ps_name)
        rospy.loginfo("Got a client! Setting parameters.")
        config = dynparamclient.update_configuration(p_dict)
        # check if it was really set
        rospy.loginfo("Parameters set: " + str(config))

        return 'succeeded'
        
if __name__=='__main__':
    # Kind of a unit test of the state
    rospy.init_node("set_dyn_params", anonymous=True)
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        sm.userdata.standard_error = 'OK'
        sm.userdata.param_server_name = "/move_base"
        sm.userdata.parameters_dict = {'planner_frequency' : 0.0}
        # Test the state using parameters
        smach.StateMachine.add(
            'SET_DYN_PARAMS_VIA_PARAMS',
            DynamicParameterSetter(param_server_name="/move_base",
                                   parameters_dict={'planner_frequency' : 0.0}),
            transitions={'succeeded':'SET_DYN_PARAMS_VIA_USERDATA'})
        # Test the state using userdata
        smach.StateMachine.add(
            'SET_DYN_PARAMS_VIA_USERDATA',
            DynamicParameterSetter(),
            transitions={'succeeded':'succeeded'})

    # This is for the smach_viewer so we can see what is happening, rosrun smach_viewer smach_viewer.py it's cool!
    sis = smach_ros.IntrospectionServer(
        'set_dyn_params', sm, '/SET_DYN_PARAMS')
    sis.start()

    sm.execute()

    sis.stop()
    
    
    