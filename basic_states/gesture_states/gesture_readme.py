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

	gesture_recognition: GestureRecognition (StateMachine)
		"""
		The robot search for a gesture and check if is the gesture that we are looking for. 
		In case that the gesture is correct, it return succeeded and the Pose where the person is. 
		Otherwise, it returned aborted. 
		You can input the gesture_name in init or in the userdata.gesture_name

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		Input keys:
		gesture_name: string with the gesture name that you search
		Output keys:
		gesture_detected: Gesture.msg. It contains all the info of the gesture detected.  
		No io_keys.

		Nothing must be taken into account to use this SM.
		""" 
