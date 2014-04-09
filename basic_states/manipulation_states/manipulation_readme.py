# Manipulation_states

	manip_to_joint_pose: manip_to_joint_pose (StateMachine)
		"""
		Executes a SM that makes the upper body movement of the robot.
	    It needs a joint goal and moves the body part to that goal.
	    Joint Goal: A goal for the joint name to move.
	    It uses MoveIt! A software for body manipulation. It analyzes the best path to reach to the pose/goal.

		Required parameters: None

		Optional parameters: None

		Input keys:
		@key manip_joint_names: indicates the joints name which we want to move.
		@key manip_joint_group: indicates the joints group name (right_arm, right_arm_torso, right_arm_torso_grasping, 
				        left_arm, left_arm_torso, left_arm_torso_grasping, head, both_arms, both_arms_and_head,
				        right_hand, left_hand)
		@key manip_goal_pose: indicates the pose to reach for joints specified in manip_joint_names
		    
		 
		Output keys:
		@key standard_error: Error
		"""

	manip_to_pose: manip_to_pose (StateMachine)
		"""
		Executes a SM that makes the upper body movement of the robot.
		It needs a 3D pose goal and moves the body part to that goal.
		It uses MoveIt! A software for body manipulation. It analyzes the best path to reach to the pose/goal.    
		Required parameters: None

		Optional parameters: None

		Input keys:
		manip_group: indicates the group which we want to move. It can be these different groups:
		    both_arms, both_arms_torso, 
		    left_arm, left_arm_torso, left_hand, --> End effector/link : hand_left_grasping_frame 
		    right_arm, right_arm_torso, right_hand --> End effector/link : hand_right_grasping_frame
		    
		manip_goal_pose: indicates the pose to reach for the group specified in manip_group, 3D pose
		    type: Point() --> x,y,z
		 
		Output keys:
		standard_error: Error
		"""

	move_hands: move_hands (StateMachine)
		"""
		This SM moves a the HAND group of joints.

		Required parameters:
		@param move_joint_group_in: indicates the controller associated with the joints

		Optional parameters: None

		Output keys:
			@key standard_error: Error 

		Input keys:
		@key move_hand_side: indicates the side of the hand
		@key move_hand_pose: indicates poses for the joints to reach : 3 joints [thumb, middle, index]

		Joints:
		[0.1, 0.1, 0,1] = Open Hands 
		"""

	move_hands_form: move_hands_form (StateMachine)
		"""
		This SM executes the "move_hands" SM from the Manipulation_States.
		It gets a name of a defined hand_pose name, this set of predefined poses are:
		- "full_open" : Hand is fully open
		- "grasp" : Hand is completely closed
		- "pre_grasp" : Hand has the thumb open, and two other fingers closed
		Parameters:
		@param: hand_pose_name:
		    "full_open", "grasp", "pre_grasp"
		@param: hand_side:
		    "left", "right"

		No input_keys

		No output_keys
		"""
	
	move_head: move_head (StateMachine)
		"""
		This SM moves the HEAD group of joints.
    
    	Required parameters: None
    
	    Optional parameters: None
	    
	    Output keys: 
	        @key standard_error: Error 
	    
	    Input keys:
	        @key move_head_pose: indicates poses for the joints to reach : 2 joints [head1, head2]
	        
	        Range: [-1,1] 
			Range: [-1,1] 
		"""

	move_joints_group: move_joints_group (StateMachine)
		"""
		This SM moves a group of joints from a group controller. 
	    It Does Not use the MoveIt Algorithm

	    Required parameters: 
	        @param move_joint_group_in: indicates the controller associated with the joints 
	    
	    Optional parameters: None
	    
	    Input keys:
	        @key move_joint_group: indicates the controller associated with the joints
	        @key move_joint_list: indicates the joints to control/move
	        @key move_joint_poses: indicates the pose/s for each joint              
	         
	    Output keys:
	        @key standard_error: Error
		"""

	play_motion_sm: play_motion_sm (StateMachine)
		"""
		This is the play_motion_sm. This SM executes the Play_motion functionality, 
		which specifies the position of the different joints in the time.

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters


		Input Keys:
		@key manip_motion_to_play: specifies the motion (from a predefined set of motions in a .yaml file)
		@key manip_time_to_play: specifies the time to reach the motion. If exceeded an error is produced.

		Output Keys:
		@key standard_error: Specifies an error output occurred in the SM.
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

	moveit_pose_goal: 
		"""
		Snippet of code on how to send a MoveIt! move_group goal to an arm in joint space

		Moveit actionserver: /move_group/goal
		Type of message: moveit_msgs/MoveGroupGoal

		Groups of REEM and their end effectors:

		right_arm -> arm_right_tool_link
		right_arm_torso -> arm_right_tool_link
		right_arm_torso_grasping -> hand_right_grasping_frame

		left_arm -> arm_left_tool_link
		left_arm_torso -> arm_left_tool_link
		left_arm_torso_grasping -> hand_left_grasping_frame

		Other groups: both_arms, head, right_hand, left_hand

		"""	


	ask_give_object_grasping(StateMachine):
		"""
	    Executes a SM that: 
       		Asks the person to give a object ['object_to_grasp']. 
	        The robot will extend its arm and open its hand.

	    It should be called only when the --Grasp-- of the object has failed,
	    which includesd the object_detection and the grasping per sé.
	        
	    Required parameters: None
	    
	    Optional parameters: None
	    
	    Input keys:
	        @key object_to_grasp: indicates the object's name we want to grasp.            
	         
	    Output keys:
	        @key standard_error: Error
		"""