# Face mock
	
	rosrun person_detection follow_me_mock.py
		"""
		Follow me Mock 

		Run the topic of face recognition:
			/people_tracker/person

		"""

	rosrun person_detection person_recognition.py
		"""
		Person detection Mock 

		Run the topic of face recognition:
			/pal_person/recognizer

		"""

# Person states
	
	learn_person: LearnPerson (StateMachine)
