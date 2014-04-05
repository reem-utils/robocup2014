# Speech mock

	rosrun asr_mock asr_srv.py
		"""
		Speech Mock. Use for listen 

		Run the Services of face recognition:
			/asr_server

		Run the topic of face recognition:
			/asr_event

		We need the file tags.txt in folder asr_mock. In this file we put the differents tags that we want that the robot recognize. 
		We can input one sentence to simulate what the robot listen. 
		"""
	
	rosrun tts_mock tts_as.py	
		"""
		Speech Mock. Use for speak 

		Run the Server of face recognition:
			/sound

		"""
# Speech_states:

	Asr_status: AsrStatus (StateMachine)
		"""
		Return the Status from ASR Server. The params that shows are:
		Active, Enabled_grammar, Language, Err_msg, War_msg.
		It shows the status on screen too.
	    
	    	@output string asr_status: Contains the status from ASRServer
	   	@output string standard_error: Shows the info from err_msg
		"""

	Say: text_to_say (StateMachine)
		"""
		To use say you need to indicate the text to be said. By default it waits 0 seconds before
		speaking and uses en_US language and don't use the nsecs wait option.

		smach.StateMachine.add(
			    'SaySM',
			    text_to_say("I'm working"),
			    transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

		@param text: the text to say
		@param text_cb: a callback returning the text to speak
		@param wait_before_speaking: how long to wait before speaking
		@param lang: The language that use to speak
		"""
	
	Activate_asr: ActivateASR (StateMachine)
	   	"""
		This state machine activate the ASR Server. It needs a grammar.
	    
		smach.StateMachine.add('ActivateASRTest',
				ActivateASR("JungleParty"),
				transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
	    
		@input string grammar_name
	  	"""

	Deactivate_asr: DeactivateASR (StateMachine)
		"""
		This state machine deactivate the ASR Server. It needs a grammar.

		smach.StateMachine.add('ActivateASRTest',
			    DeactivateASR("JungleParty"),
			    transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

		@input string grammar_name

		"""

	Read_asr: ReadASR  (StateMachine)
		"""
		Read from asr_event and returns what the user has said

		@output string asr_userSaid
		@output actiontag[] asr_userSaid_tags

		"""

	ListenTo: ListenToSM (StateMachine)
		"""      
		This function will ListenTo a Grammar and get the interesting arg[] for that given grammar
		Right now it just puts on asr_userSaid what the robot has heard without further analysis

		smach.StateMachine.add('ListenToTest',
				ListenToSM("JungleParty"),
				transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

		@input string grammar_name
		@output string asr_userSaid
		@output actiontag[] asr_userSaid_tags
		"""

	ask_question: AskQuestionSM (StateMachine)
		"""      
		This function will do a question, listen the answer and then confirm it.
		If return succeeded, the confirm answer was yes. Otherwise, it return aborted

		@input string text or tts_text
		@output string asr_userSaid
		@output actiontag[] asr_userSaid_tags
		"""

	listen_and_repeat: ListenRepeatSM (StateMachine)
		"""
		Executes a SM that learns one face. 
		The robot listen the person and later repeat. It needs a grammar. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		No input keys.
		No output keys.
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""

	say_yes_or_no: SayYesOrNoSM (StateMachine)
		"""
		Executes a SM that learns one face. 
		The robot listen the person and returns succeeded in case that he said yes
		and aborted if said no. 

		Required parameters:
		No parameters.

		Optional parameters:
		No optional parameters

		No input keys.
		No output keys.
		No io_keys.

		Nothing must be taken into account to use this SM.
		"""    
