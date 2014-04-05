# Util_states:

	Sleeper: Sleeper (State)
		"""
		Sleeper State

		Waits the specified time

		Input Keys:
			sleeper_time: Time we want to sleeper
		Output Keys:
			standard_error: String thats shows what fails
		"""

	Timeout: TimeOut (State)
		"""
		TimeOut State

		Returns succeeded if the specified time has passed. 

		Input Keys:
			wait_time: Time we want to wait
		Output Keys:
			standard_error: String thats shows what fails    
		"""
	topic_reader: topic_reader (StateMachine)
		"""
		Executes a SM that reads a specified topic.

		Required parameters:
		@param topic_name: Name of the topic (e.g. \amcl_pose) 
		@type String: 
		@param topic_type: Class of the topic (e.g. PoseStamped)
		@type Class: 
		@param topic_time_out: Number of seconds to 'wait' at maximum to finish the SM
		@type Float: 

		Optional parameters:
		No optional parameters


		No input keys.

		Output keys:
		@keyword topic_output_msg: Output of the topic
		@type: topic_type:
		@keyword standard_error: Specifies an error string if occurred.
		@type String: 


		No io_keys.

		Example of usage:
		topic_name = '/sonar_base'
		topic_type = Range
		topic_time_out = 60

		smach.StateMachine.add(
		    'dummy_state',
		    topic_reader(topic_name,topic_type,topic_time_out),
		    transitions={'succeeded': 'Printing','preempted':'preempted', 'aborted':'aborted'})
		"""

	pose_at_distance:
		pose_at_distance(pose,distance):
			"""
			Returns a pose that has the same orientation as the original
			but the position is at a distance from the original.
			Very usefull when you want to mantain a distance from an object. 
			"""
		pose_at_distance2(pose1,pose2,distance):
			"""
			Returns a pose that has the same orientation as the original
			but the position is at a distance from the original.
			Very usefull when you want to mantain a distance from an object. 
			It takes into account where the robot is
			"""
		
	math_utils: 
		xy_with_angle(alfa, distance):
			"""
			Returns the x and y components for a 2D vector that
			has that angle ( radians) reffered to the horizontal and has the
			magnitude distance.
			"""

		vector_magnitude2D(vec):
			"""
			Returns the magnitude of the given vector.
			"""

		normalize_vector2D(vec):
			"""
			Returns a normalized (unitary) copy of *vec*.
			"""

		multiply_vector2D(vec, factor):
			"""
			Multiplies each component of *vec* by the given *factor*.
			There is the factor zero condition, to aviod having variables
			that were negative having a value or -0.0.
			"""

		vector_magnitude(vec):
			"""
			Returns the magnitude of the given vector.
			"""
	
		normalize_vector(vec):
			"""
			Returns a normalized (unitary) copy of *vec*.
			"""

		multiply_vector(vec, factor):
			"""
			Multiplies each component of *vec* by the given *factor*.
			There is the factor zero condition, to aviod having variables
			that were negative having a value or -0.0.
			"""

		add_vectors(vec1, vec2):
			"""
			Element-wise addition of vec1 and vec2.
			"""

		substract_vector(vec1, vec2):
			"""
			Element-wise substraction of vec2 from vec1.
			"""
 
		multiply_quaternions(a, b):
			"""
			Multiplies quaternions `a' and `b`. Remember that this operation
			is noncommutative.
			"""

		cross_product(a, b):
			"""
			Makes the cross product of two vectors
			"""

		dot_product(a, b):
			"""
			Makes a dot product of two vectors
			Returns a number
			"""
		

