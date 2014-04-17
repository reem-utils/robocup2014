# Mock server

	rosrun grasping_mock object_detection_mock.py
		"""
		This is a Service for Object Recognition.

		It publishes the messages (of type 'ObjectDetection') to the topic '/object_detect/recognizer'.
		"""

# Object grasping states

	detect_object_sm: detect_object (StateMachine)
		"""
		Executes a SM that subscribe in a topic and return what objects are detected.
		It call a Recognizer Service, it forces the start,
		It doesn't close the recognizer


		Required parameters : 
		No parameters.

		Optional parameters:
		     minConfidence, is the value to filter the object
		No optional parameters


		input keys: 

		output keys:
		standard_error: inform what is the problem
		object: ObjectDetectionMessage
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

	get_object_information: GetObjectInfoSM (StateMachine)
		"""
		Executes a SM that search for object. 
		Given the object name, it search which place is the most probably that we can find it. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Input keys:
			object_name: string with the object's name
		Output keys:
			object_location: string with the place most probably  
			standard_error: String that show what kind of error could be happened
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	recognize_object: recognize_object (StateMachine)
		"""
		Executes a SM that look if it can recognize object

		It have 2 options:
		if you complete the name, it will return if
		    it find this object, and return the object message of it.
		If you don't complete it will return the object with more confidence.


		Required parameters : 
		No parameters.

		Optional parameters:
		    object_name, of the person that you are looking for, it will return
				aborted if can't find 

		input keys:
		    object_name, it's optional of the person we are looking for, it can be the name or ""
		output keys:
		    standard_error: inform what is the problem
		    objectd, is a message that have ObjectDetection, 
			it will be None if can't find any faces
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	search_object: SearchObjectSM (StateMachine)
		"""
		Executes a SM that search for object. 
		Given the object name, it search which place is the most probably that we can find it. 
		It goes to the place and start the object recognition. It returns the ObjectDetect where the object is. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Input keys:
		object_to_grasp: string with the object's name
		Output keys:
		object_pose: Pose with the object's coordinates  
		standard_error: String that show what kind of error could be happened
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
