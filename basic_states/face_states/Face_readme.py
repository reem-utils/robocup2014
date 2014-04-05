# Face mock:

	rosrun face_detection face_detection.py
		"""
		Face recognition Mock service 

		Run the Services of face recognition:
			/pal_face/recognizer
			/pal_face/start_enrollment
			/pal_face/stop_enrollment
			/pal_face/set_database

		Run the topic of face recognition:
			/pal_face/recognizer

		If you press a key, it learn a new face.
		Always is the same face, if you want to change the values need to modify the code. 
		"""
# Face_states:

	ask_name_learn_face: SaveFaceSM (StateMachine)
		"""
		Executes a SM that learns one face. 
		The robot listen the name from the person and then learns his face. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		No input keys.
		No output keys.
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	Detect_faces: detect_face (StateMachine)
		"""
		Executes a SM that subscribe in a topic and return what faces
		are detect.
		It call a Recognizer Service, it force the start,
		It doesn't close the recognizer

		Required parameters : 
		No parameters.

		Optional parameters:
		minConfidence, is the value to filter the face 

		No input keys.       
		output keys:
		standard_error: inform what is the problem
		faces: is a message that have array of face FaceDetection
		No io_keys.

		"""
	Drop_faces: drop_faces (StateMachine)
		"""
		Executes a SM that manage the database.s
		Only do a service call with name and purge option
		provided for input_key 

		Required parameters: 
		No parameters.

		Optional parameters: learning_time, by default is 5 seconds

		input keys: name, it's the name of the dataBaser
			purgeAll, “False” parameter means that we do not want to empty the database. Set to “True” in
				    order to remove previously stored faces.

		output keys: standard_error: string with the error, now we don't complete
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	Learn_face: learn_face (StateMachine)
		"""
		Executes a SM that does the process off enrollment.
		It call a enrollmentStard Service,
		it waits learning_time,
		and then stops the enrollment, 


		Required parameters : 
		No parameters.

		Optional parameters: 
		learning_time, by default is 5 seconds

		input keys: 
		name, it's the name of the person who will enroll
		output keys: 
		standard_error, string with the error
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	Recognize_face: recognize_face (StateMachine)
		"""
		Executes a SM that look if it can recognize faces

		It have 2 options:
		if you complete the name, it will return if
		it find this face, and return the face message of it.
		If you don't complete it will return the face with more confidence.


		Required parameters : 
		No parameters.

		Optional parameters:
		name, of the person that you are looking for, it will return
		aborted if can't find 

		input keys:
		Name, it's optional of the person we are looking for, it can be the name or ""
		output keys:
		standard_error: inform what is the problem
		face, is a message that have FaceDetection, 
		it will be None if can't find any faces
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""
	Searching_person: searching_person (StateMachine)
		""" 
		This state machine only can return succeeded, it will try to
		find a face all the time, if it find it will return succeeded.
		It can return aborted if the time live.


		Required parameters : 
		No parameters.

		Optional parameters:
			    name, of the person that you are looking for
			    max time ?¿ maybe it can be interesting to put a maximum time

		input keys:
		    name, it's optional of the person we are looking for,
		
		output keys:
		    standard_error: inform what is the problem
		    face, if it find it will return the face
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

