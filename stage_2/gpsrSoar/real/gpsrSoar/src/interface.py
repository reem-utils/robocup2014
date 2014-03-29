#! /usr/bin/env python

import roslib; roslib.load_manifest('gpsrSoar')
import rospy
import sys
import smach
import smach_ros
import actionlib
import time



"""sys.path.append("/home/albert/fuerte_workspace/reem_at_iri/state_machines/common/src")"""

# from pal_smach_utils.utils.find_and_recognize_people import FindAndRecognizeParticularPersonStateMachine as FindAndRecognizeParticularPersonSM
# from pal_smach_utils.object_finding_algorithms.ofb_first_approach import OFBFirstApproach as OFBFirstApproachSM

#from pal_smach_utils.grasping.sm_grasp import GraspStateMachine as GraspSM
#from pal_smach_utils.navigation.move_to_room import MoveToRoomStateMachine as MoveToRoomSM
# from follow_me import FollowMe as FollowMeSM
# from follow_me import LearnPerson as LearnPersonSM
#from pal_smach_utils.grasping.sm_release import ReleaseObjectStateMachine as ReleaseSM
#from pal_smach_utils.grasping.arm_and_hand_goals import get_pose_for_arm_in_front as delvierpos

#New functions added
# from pal_smach_utils.utils.learn_face import LearnFaceStateMachine as LearnPersonSM
# from pal_smach_utils.utils.recognize_face import RecognizeFaceStateMachine as RecognizePersonSM
#from pal_smach_utils.utils.introduce_yourself import IntroduceYourselfStateMachine as IntroduceSM
#from pal_smach_utils.grasping.search_object_with_confidence import SearchObjectWithConfidenceStateMachine as FindObjectSM

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

SOAR_GP_PATH = roslib.packages.get_pkg_dir("gpsrSoar") + "/SOAR/gp.soar"

def call_go_to(loc_name):
	print "SM : go_to %s" % (loc_name)
	'''
	mr = MoveToRoomSM()
	mr.userdata._data = {'room_name': loc_name}
	print mr.userdata.keys()
	out = mr.execute()
	return out
	'''
	time.sleep(3)
	return "succeeded"

def call_exit(): #TODO well
	print "SM : go_to exit"
	'''
	mr = MoveToRoomSM()
	mr.userdata._data = {'room_name': 'exit'}
	print mr.userdata.keys()
	out = mr.execute()
	return out
	'''
	time.sleep(3)
	return "succeeded"

def call_learn_person():
	print "SM : learn_person"
	'''
	lp = LearnPersonSM()
	out = lp.execute()
	#userdata now contains only the name in userdata.out_person_name
	#Revise in pal_smach_utils/src/pal_smach_utils/utils/learn_face
	PersonName = lp.userdata.out_person_name
	# PT_id_of_person = lp.userdata.PT_Id_of_person
	# LP_all_person_data = lp.userdata.LP_all_person_data
	# return (out, PT_id_of_person, LP_all_person_data)
	return (out, PersonName)
	'''
	time.sleep(3)
	return "succeeded"

def call_recognize_person(): #TODO
	'''
	rp = RecognizePersonSM()
	out = rp.execute()
	PersonName = rp.userdata.out_person_name
	return (out, PersonName)
	print "SM : recognize_person" 
	'''
	time.sleep(3)
	return "succeeded"

def call_introduce(): #TODO 
	print "SM : introduce" 
	'''
	intro = IntroduceSM()
	out = intro.execute()
	return out
	'''
	time.sleep(3)
	return "succeeded"

def call_point_at(): #TODO
	print "SM : point_at" 
	return "succeeded"

# def call_follow(): #TODO
# 	print "SM : follow" 
# 	f = FollowMeSM()
# 	out = f.execute()
# 	return out

def call_find_object(object_name, loc_name):
	print "SM : find_object %s" % (object_name)
	'''
	fo = OFBFirstApproachSM(target_object_key='target_obj')
	fo.userdata.target_obj = object_name
	fo.userdata.in_room_name= loc_name
	out = fo.execute()
	found_object = fo.userdata.out_object_found
	return (out, found_object)
	'''
	time.sleep(3)
	return "succeeded"

def call_grasp(obj):
	"""print "SM : grasp %s" % (obj)
	grasp = GraspSM()
	grasp.userdata.pose_object = obj
	out = grasp.execute()
	return out"""
	time.sleep(3)
	return "succeeded"

def call_find_person(person_name):
	print "SM : find_person %s" % (person_name)
	'''
   	fp = FindAndRecognizeParticularPersonSM(name_key=person_name)
   	# fp.userdata._data = {'name_key': person_name}

   	out = fp.execute()
   	return out
   	'''
   	time.sleep(3)
   	return "succeeded"

def call_bring_to(person_name): #Solo hace release
	print "SM : bring_to %s" % (person_name)
	'''
	r = ReleaseSM()
	r.userdata.releasing_position = delvierpos();
	out = r.execute()
   	return out
   	'''
   	time.sleep(3)
   	return "succeeded"

def call_ask_name():
	print "SM : ask_name" 
	time.sleep(3)
   	return "succeeded"

def call_introduce_me():
	print "SM : introduce_me" 
	time.sleep(3)
   	return "succeeded"

def call_memorize_person():
	print "SM : memorize_person"
	time.sleep(3)
	return "succeeded"

def trad_obj(obj):
	obj = int(obj)
	if obj == 0:
		return "coke"
	else:
		return "NULL"

def trad_pers(pers):
	pers = int(pers)
	if pers==0:
		return "albert"
	else:
		return "NULL"

def trad_loc(loc):
	loc = int(loc)
	if loc == 0:
		return "charger"
	elif loc == 1:
		return "kitchen"
	elif loc == 2:
		return "business"
	elif loc == -1:
		return "exit"
	else:
		return "NULL"

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
	print "New goal"
	kernel = create_kernel()
	agent = create_agent(kernel, "agent")
	agent_load_productions(agent,SOAR_GP_PATH)		

	# cmd = 'learn --on'
	# res = kernel.ExecuteCommandLine(p_cmd, agent.GetAgentName)
	# p_cmd = 'watch --chunk --print'
	# res

	goal_achived = False
	while not goal_achived:
		#agent.Commit()

		agent.RunSelfTilOutput()
		print "El agente ha generado comandos de salida"

		#agent.Commands()
		numberCommands = agent.GetNumberCommands()
		print "Numero de comandos recibidos del agente: %s" % (numberCommands)

		if numberCommands == 0:
			print 'Se han recibido 0 comandos, aborting!!'
			return 'aborted'

		i=0
		while i<numberCommands:
			command = agent.GetCommand(i)
			command_name = command.GetCommandName()
			print "El nombre del commando %d/%d es %s" % (i+1,numberCommands,command_name)

			out = "NULL"
			if command_name == "navigate":
				loc_to_navigate = command.GetParameterValue("loc")
				loc = trad_loc(loc_to_navigate)
				print loc
				if (loc =="NULL"):
					print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)

				out = call_go_to(loc)

			elif command_name == "grasp":
				obj_to_grasp = command.GetParameterValue("obj")
				obj = trad_obj(obj_to_grasp)
				print obj
				if (obj =="NULL"):
					print "ERROR: el objeto %s no existe" % (obj_to_grasp)

				out = call_grasp(obj)

			elif command_name == "deliver":
				to_pers = command.GetParameterValue("pers")
				pers = trad_pers(to_pers)
				print pers
				if (pers =="NULL"):
					print "ERROR: la persona %s no existe" % (to_pers)

				out = call_bring_to(pers)

			elif command_name == "search-object":
				'''
				obj_to_search = command.GetParameterValue("obj")
				obj = trad_obj(obj_to_search)
				print obj
				if (obj =="NULL"):
					print "ERROR: el objeto %s no existe" % (obj_to_search)

				out,obj = call_find_object(obj,"asd")
				'''
				out = "succeeded"

			elif command_name == "search-person":
				'''
				pers_to_search = command.GetParameterValue("pers")
				pers = trad_pers(pers_to_search)
				print pers
				if (pers =="NULL"):
					print "ERROR: la persona %s no existe" % (pers_to_search)
				
				out = call_find_person(pers)
				'''
				out = "succeeded"

			elif command_name == "ask-name":
				out = call_ask_name()

			elif command_name == "follow":
				#out = call_follow()
				print "call_follow does not exist"

			elif command_name == "introduce-me":
				out = call_introduce_me()

			elif command_name == "memorize-person":
				out = call_memorize_person()

			elif command_name == "recognize-person":
				out = call_recognize_person()

			elif command_name == "exit-apartment":
				loc_to_navigate = command.GetParameterValue("loc")
				loc = trad_loc(loc_to_navigate)
				print loc
				if (loc =="NULL"):
					print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)

				out = call_go_to(loc)

			elif command_name == "achived":
				goal_achived = True
				out = "succeeded"

			else:
				print "ERROR: El commando %s no existe" % (command_name)
				command.AddStatusComplete()

			
			print "SM return: %s" % (out)
			if out=="succeeded": 
				print "SM succeeded"
				command.AddStatusComplete()

			else:
				print "gpsrSoar interface: unknown ERROR"
				exit(1)

			i+=1

		#kernel.CheckForIncomingCommands()

	command.AddStatusComplete()
	return 'succeeded'


	kernel.DestroyAgent(agent)
	kernel.Shutdown()
	del kernel


if __name__ == "__main__":
	main()
