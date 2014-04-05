# Navigation_states:

	enter_room: EnterRoomSM (StateMachine)
		"""
		Executes a SM that enter a room. 
		Wait if the door isn't open and enter the room.

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Input keys:
		nav_to_poi_name: indicates the point we need to reach after detect that the door is open
		Output keys:
		standard_error: String that show what kind of error could be happened
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

	get_current_robot_pose: get_current_robot_pose (StateMachine)
		"""
		Return the actual position and the yaw of the robot

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		No input keys.
		Output keys:
		current_robot_pose: PoseWithCovariance that show the position of the robot 
		current_robot_yaw: Indicates in radians the yaw of the robot
		standard_error: String that show what kind of error could be happened
		No io_keys.
		"""

	nav_go_forward: goForwardSM (StateMachine)
		"""
		Executes a SM that go forward respect the actual position. 
		It has a little error with yaw. 

		TODO: It's a beta test

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Inputs keys:
		distance: The meters that the robot go forward

		No output keys.
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

	nav_to_coord: nav_to_coord (StateMachine)
		"""
		Navigate to given map coords.

		This SM navigates to a given coordenates using the parameter: "nav_to_coord_goal"
		This input data should be a (x,y,yaw) point
		With frame_id param you can indicate if the coord are from the map (frame_id = /map)
		or from the robot (frame_id = /base_link). By default it is /map. 

		@input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
		rotation in radians
		@output_keys: standard_error string representing the possible error
		"""

	nav_to_poi: nav_to_poi (StateMachine)
		"""
		This state machine receive the name of the point and go there. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Input keys: 
		nav_to_poi_name: String that contain the poi information
		Output keys:
		standard_error: String that show what kind of error could be happened
		No io_keys.  
		"""

	nav_to_coord_concurrent: nav_to_coord_concurrent
		"""
		Navigate to given map coords.

		This SM navigates to a given coordenates using the parameter
		This input data should be a (x,y,yaw) point
		It works like nav_to_coord, the difference is that you don need to wait the final

		@input_keys: nav_to_coord_goal type list [x, y, yaw] where x, y are float, and yaw a float representing
		rotation in radians
		@output_keys: standard_error string representing the possible error
		@optional you can put a frame_id, if you don't put it will be the /map
		"""
