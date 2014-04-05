# Mock server

	rosrun grasping_mosk object_detection_mock.py
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
