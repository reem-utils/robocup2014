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
