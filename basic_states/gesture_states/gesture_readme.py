# Gesture mock

	rosrun gesture_detection_mock gesture_detect_mock.py
		"""
		Gesture Detection Mock  

		Run the topic of face recognition:
			/gesture_detection/gesture

		Always recognize the wave gesture
		"""

# Gesture_recognition

	gesture_detection_sm: gesture_detection_sm (StateMachine)
		"""
		Executes a SM that subscribe in a topic and return what gestures are detected.

		Required parameters : 
		No parameters.

		No optional parameters


		input keys: 

		output keys:
		standard_error: inform what is the problem
		gesture_detected: 
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
