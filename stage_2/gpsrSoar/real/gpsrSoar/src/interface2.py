#! /usr/bin/env python

import roslib; roslib.load_manifest('gpsrSoar')
import rospy
import sys
import smach
import smach_ros
import actionlib
import time
from translator import idx2obj, obj2idx

from smach_ros import ServiceState, SimpleActionState
from std_srvs.srv import Empty


#TODO: find_person import FindPersonSM
#TODO: complete_grasp_pipeline import CompleteGraspPipelineStateMachine as GraspSM
#TODO: search_object_with_confidence import SearchObjectWithConfidenceStateMachine as SearchObjSM
from navigation_states import nav_to_poi #navigation.move_to_room import MoveToRoomStateMachine as MoveToRoomSM
# from pal_smach_utils.navigation.follow_and_stop import FollowAndStop as FollowMeSM
#TODO: learn_face import LearnFaceStateMachine as LearnPersonSM
#from pal_smach_utils.utils.point_at import SMPointInFront as PointAtSM
#TODO: grasping.sm_release import ReleaseObjectStateMachine as ReleaseSM
#TODO: recognize_face import RecognizeFaceStateMachine as RecognizePersonSM
#TODO: introduce_yourself import IntroduceYourselfStateMachine as IntroduceSM
#TODO: sound_action import SpeakActionState <-- listen to

#edit your path in gpsrSoar/src/pathscript.py

try:
    from pathscript import *
except ImportError:
    print "PATHSCRIPT COULDN'T BE IMPORTED!!"
    print "pathscript.py isn't in your computer. \n please, create it in: \n $roscd gpsrSoar/src/"
    print "then define: \nPATH_TO_SOAR = [PATH to the bin folder in SOAR]\nPATH_TO_STANFORD_PARSER = [PATH to the stanford parser folder]\n\npointing to these packages folders\n\n"

try:
	PATH_TO_SOAR = roslib.packages.get_pkg_dir("gpsr") + "/../soar9.3.2/bin"
	sys.path.append(PATH_TO_SOAR)
	import Python_sml_ClientInterface as sml
except ImportError:
	try:
		from pathscript import *
	except ImportError:
		print "PATHSCRIPT COULDN'T BE IMPORTED!!"
		print "pathscript.py isn't in your computer. \n please, create it in: \n $roscd gpsrSoar/src/"
		print "then define: \nPATH_TO_SOAR = [PATH to the bin folder in SOAR]\nPATH_TO_STANFORD_PARSER = [PATH to the stanford parser folder]\n\npointing to these packages folders\n\n"

	path = PATH_TO_SOAR
	sys.path.append(path)
	import Python_sml_ClientInterface as sml

SOAR_GP_PATH = roslib.packages.get_pkg_dir("gpsrSoar") + "/SOAR/gp2.soar"

def call_go_to(loc_name):
    '''
    out = aborted
	tries = 0
	while(out==aborted and tries<3):
		if loc_name == 'exit':
			out = call_exit()
		else:
			print "SM : go_to %s" % (loc_name)
			tosay = "I'm going to the "+loc_name
			speak = SpeakActionState(text=tosay)
			speak.execute(ud=None)
			mr = nav_to_poi()
			mr.userdata._data = {'nav_to_poi_name': loc_name.replace(' ', '_')}#{'room_name': loc_name.replace(' ', '_')}
			#mr.userdata.room_name = loc_name
			out = mr.execute()
		tries = tries+1

	return succeeded '''
    
    print 'call_go_to' 
    time.sleep(3)  
    return "succeeded" 
	

def call_exit(): #TODO well
    '''	out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : go_to exit"
        tosay = "I'm going to the exit"
        speak = SpeakActionState(text=tosay)
        speak.execute(ud=None)
        mr = nav_to_poi()#MoveToRoomSM()
        mr.userdata._data = {'nav_to_poi_name': 'pre_exit'}#{'room_name': 'pre_exit'} #preguntar gerard
        #mr.userdata.room_name = loc_name
        out = mr.execute()
        # sr = ServiceState('/alive_engine/stop', Empty)
        # out = sr.execute(ud=None)
        mr.userdata._data = {'nav_to_poi_name': 'exit'}#{'room_name': 'exit'}
        out = mr.execute() 
        
        tries = tries+1
    
    return succeeded ''' 

    print 'call_exit'   
    time.sleep(3)   
    return "succeeded"  

def call_learn_person():
    '''	out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : learn_person"
        tosay = "I'm going to learn the person in front of me"
        speak = SpeakActionState(text=tosay)
        speak.execute(ud=None)
        
        lp = LearnPersonSM()
        out = lp.execute()
        #PersonName = lp.userdata.out_person_name
        tries = tries+1
    
    #time.sleep(3)
    return succeeded #(out, PersonName)'''
    
    print 'call_learn_person'
    time.sleep(3)
    return "succeeded"

def call_recognize_person(): 
    '''	out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : recognize_person" 
        tosay = "I'm going to recognize the person in front of me"
        speak = SpeakActionState(text=tosay)
        speak.execute(ud=None)
        rp = RecognizePersonSM()
        out = rp.execute()
        PersonName = rp.userdata.out_person_name
        tries = tries+1
    
    return succeeded #(out, PersonName)	'''
    
    print 'call_recognize_person'
    time.sleep(3)
    return "succeeded" 

def call_introduce(): 
    '''	print "SM : introduce" 
    
    intro = IntroduceSM()
    out = intro.execute()
    return succeeded '''
    
    print 'call_introduce'
    time.sleep(3)
    return "succeeded"

def call_point_at(): #TO TEST	
    '''
    out = aborted
	tries = 0
	while(out==aborted and tries<3):
		print "SM : point_at" 
		tosay = "I'm going to point"
		speak = SpeakActionState(text=tosay)
		speak.execute(ud=None)
		sm =PointAtSM()
		out = sm.execute()
		tries = tries+1

	return succeeded
    '''
    print 'call_point_at'
    time.sleep(3)
    return "succeeded"

def call_follow(): #TODO
    print "SM : follow-me" 
    '''
    tosay = "I'm going to follow you"
    speak = SpeakActionState(text=tosay)
    speak.execute(ud=None)
    sm =FollowMeSM(0.9, 'gpsr_follow')
    out = sm.execute(ud=None)
    '''
    time.sleep(3)
    return succeeded
# 	print "SM : follow" 
# 	f = FollowMeSM()
# 	out = f.execute()
# 	return out


def call_find_object(object_name, loc_name):
    '''
    out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : find_object %s" % (object_name)
        tosay = "I'm going to search for "+object_name
        speak = SpeakActionState(text=tosay)	
        speak.execute(ud=None)
        sm = SearchObjSM()
        sm.userdata.object_to_search_for = object_name
        sm.execute()
        for i in range(len(sm.userdata.object_found.object_list)):
            try:
                sm.userdata.object_found.object_list[i].name == object_name
                return succeeded
            except IndexError:
                tries = tries+1



    if (out == aborted):
        tosayn = "Here it should be the " + object_name + " but I can't see it"
        speakn = SpeakActionState(text=tosayn)	
        speakn.execute(ud=None)
    return succeeded
    '''
    print 'call_find_object'
    time.sleep(3)
    return "succeeded"


def call_grasp(obj):
    '''
    out = aborted
    tries = 0
    while(out==aborted and tries<5):
        print "SM : grasp %s" % (obj)
        tosay = "I'm going to grasp the " + obj
        speak = SpeakActionState(text=tosay)
        
        speak.execute(ud=None)
        grasp = GraspSM()
        grasp.userdata._data = {'object_to_search_for': obj, 'ask_for_help_key': False}
        out = grasp.execute()
        tries = tries+1

    return succeeded
    '''
    print 'call_grasp'
    time.sleep(3)
    return "succeeded"


def call_find_person(person_name):
    # print "SM : find_person %s" % (person_name)
    # tosay = "I'm going to search for a person" #+person_name
    # speak = SpeakActionState(text=tosay)
    # speak.execute(ud=None)
#   	fp = FindPersonSM()
#   	out = fp.execute()
#   	#found_person = fp.userdata.closest_person
#   	return succeeded
    print 'call_find_person'
    time.sleep(3)
    return "succeeded"

def call_bring_to(person_name): #Solo hace release TODO
    '''
    out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : bring_to %s" % (person_name)
        tosay = "Take it please"
        speak = SpeakActionState(text=tosay)
        speak.execute(ud=None)
        r = ReleaseSM()
        r.userdata.releasing_position = None;
        out = r.execute()
    
    return succeeded
    
    '''
    print 'call_bring_to'
    time.sleep(3)
    return "succeeded" 

def call_ask_name():
    '''
    print "SM : ask_name" 
    return call_learn_person()
    '''
    print 'call_ask_name'
    time.sleep(3)
    return "succeeded"

def call_introduce_me():
    '''
    out = aborted
    tries = 0
    while(out==aborted and tries<3):
        print "SM : introduce_me" 
        intr = IntroduceSM()
        out = intr.execute()
        tries = tries+1

    return succeeded
    '''
    print 'call_introduce_me'
    time.sleep(3)
    return "succeeded"


def define_prohibitions(): #TODO
	pass

def create_kernel():
	kernel = sml.Kernel.CreateKernelInCurrentThread()
	if not kernel or kernel.HadError():
		print kernel.GetLastErrorDescription()
		exit(1)
	return kernel

def create_agent(kernel, name):
	agent = kernel.CreateAgent("agent")
	if not agent:
		print kernel.GetLastErrorDescription()
		exit(1)
	return agent

def agent_load_productions(agent, path):
	agent.LoadProductions(path)
	if agent.HadError():
		print agent.GetLastErrorDescription()
		exit(1)


def main():
	print "******************************\n******************************\nNew goal\n******************************\n******************************\n"
	first_time = time.time()
	kernel = create_kernel()
	agent = create_agent(kernel, "agent")
	agent_load_productions(agent,SOAR_GP_PATH)		
	agent.SpawnDebugger()

	# p_cmd = 'learn --on'
	# res = agent.ExecuteCommandLine(p_cmd)
	# res = kernel.ExecuteCommandLine(p_cmd, agent.GetAgentName)
	kernel.CheckForIncomingCommands()
	p_cmd = 'watch --learning 2'
	res = agent.ExecuteCommandLine(p_cmd)
	print res
	

	goal_achieved = False
	while not goal_achieved:
		# if (time.time()-first_time > 300):
		# 	return aborted

		agent.Commit()

		agent.RunSelfTilOutput()
		print "El agente ha generado comandos de salida"

		agent.Commands()
		numberCommands = agent.GetNumberCommands();
		print "Numero de comandos recibidos del agente: %s" % (numberCommands)

		i=0;
		if numberCommands == 0:
			print 'KABOOOOOOOOOOOOOOOOOOM!!!!!!!!!!!!!!!'
			return 'aborted'
		else:
			while i<numberCommands:
				command = agent.GetCommand(i)
				command_name = command.GetCommandName()
				print "El nombre del commando %d/%d es %s" % (i+1,numberCommands,command_name)

				out = "NULL"
				if command_name == "navigate":
					loc_to_navigate = command.GetParameterValue("loc")
					# loc = trad_loc(loc_to_navigate)
					# print str(loc_to_navigate)
					loc = idx2obj(int(loc_to_navigate), 'LOCATIONS')
					print loc
					if (loc =="NULL"):
						print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)

					out = call_go_to(loc)

				elif command_name == "grasp":
					obj_to_grasp = command.GetParameterValue("obj")
					# obj = trad_obj(obj_to_grasp)
					obj = idx2obj(int(obj_to_grasp),'ITEMS')
					print obj
					if (obj =="NULL"):
						print "ERROR: el objeto %s no existe" % (obj_to_grasp)

					out = call_grasp(obj)

				elif command_name == "deliver":
					to_pers = command.GetParameterValue("pers")
					# pers = trad_pers(to_pers)
					pers = idx2obj(int(to_pers),'PERSONS')
					print pers
					if (pers =="NULL"):
						print "ERROR: la persona %s no existe" % (to_pers)

					out = call_bring_to(pers)

				elif command_name == "search-object":
					
					obj_to_search = command.GetParameterValue("obj")
					obj = idx2obj(int(obj_to_search), 'ITEMS')
					print obj
					if (obj =="NULL"):
						print "ERROR: el objeto %s no existe" % (obj_to_search)

					out = call_find_object(obj,"asd")
					
				elif command_name == "search-person":
					pers_to_search = command.GetParameterValue("pers")
					pers = idx2obj(int(pers_to_search), 'PERSONS')
					print pers
					if (pers =="NULL"):
						print "ERROR: la persona %s no existe" % (pers_to_search)
					
					out = call_find_person(pers)
					

				elif command_name == "point-obj":
					out = succeeded #call_point_at()

				elif command_name == "ask-name":
					out = call_ask_name()

				elif command_name == "follow":
					out = call_follow()

				elif command_name == "introduce-me":
					out = call_introduce_me()

				elif command_name == "memorize-person":
					out = call_learn_person()

				elif command_name == "recognize-person":
					out = call_recognize_person()

				# elif command_name == "exit-apartment":
				# 	loc_to_navigate = command.GetParameterValue("loc")
				# 	loc = trad_loc(loc_to_navigate)
				# 	print loc
				# 	if (loc =="NULL"):
				# 		print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)

				# 	out = call_go_to(loc)

				elif command_name == "achieved":
					goal_achieved = True
					out = "succeeded"

				else:
					print "ERROR: El commando %s no existe" % (command_name)
					command.AddStatusComplete()

				
				print "SM return: %s \n\n" % (out) 
				if out=="succeeded": 
					command.AddStatusComplete()

				else:
					print "gpsrSoar interface: unknown ERROR"
					exit(1)

				i+=1


	command.AddStatusComplete()
	return 'succeeded'


	kernel.DestroyAgent(agent)
	kernel.Shutdown()
	del kernel


if __name__ == "__main__":
	main()
