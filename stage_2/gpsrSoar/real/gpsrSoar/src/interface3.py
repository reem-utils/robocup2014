#! /usr/bin/env python

import roslib; roslib.load_manifest('gpsrSoar')
import rospy
import sys
import smach
import smach_ros
import actionlib
import time
from translator import idx2obj, obj2idx
from pal_smach_utils.speech.sound_action import SpeakActionState

succeeded = 'succeeded'


"""sys.path.append("/home/albert/fuerte_workspace/reem_at_iri/state_machines/common/src")"""

# from pal_smach_utils.utils.find_person import FindPersonStateMachine as FindPersonSM
# from pal_smach_utils.object_finding_algorithms.object_finding_behaviours import ObjectFindingBehaviour as SearchObjSM

# from pal_smach_utils.grasping.complete_grasp_pipeline import CompleteGraspPipelineStateMachine as GraspSM
# from pal_smach_utils.navigation.move_to_room import MoveToRoomStateMachine as MoveToRoomSM
# from follow_me import FollowMe as FollowMeSM
# from pal_smach_utils.utils.learn_face import LearnFaceStateMachine as LearnPersonSM
# from pal_smach_utils.grasping.sm_release import ReleaseObjectStateMachine as ReleaseSM

# from pal_smach_utils.utils.recognize_face import RecognizeFaceStateMachine as RecognizePersonSM
# from pal_smach_utils.utils.introduce_yourself import IntroduceYourselfStateMachine as IntroduceSM


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
    print "SM : go_to %s" % (loc_name)

    tosay = "I'm going to the "+loc_name
    SAS = SpeakActionState(text=tosay)
    out = SAS.execute(ud=None)
    
    # mr = MoveToRoomSM()
    # # mr.userdata._data = {'room_name': loc_name}
    # mr.userdata.room_name = loc_name

    # out = mr.execute()
    # return out
    
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
    
    # lp = LearnPersonSM()
    # out = lp.execute()
    # PersonName = lp.userdata.out_person_name

    # #time.sleep(3)
    # return (out, PersonName)
    
    time.sleep(3)
    return "succeeded"

def call_recognize_person(): 
    print "SM : recognize_person" 

    # rp = RecognizePersonSM()
    # out = rp.execute()
    # PersonName = rp.userdata.out_person_name
    # return (out, PersonName)
    
    # '''
    time.sleep(3)
    return "succeeded" 

def call_introduce(): 
    print "SM : introduce" 
    # intro = IntroduceSM()
    # out = intro.execute()
    # return out
    # '''
    time.sleep(3)
    return "succeeded"

def call_point_at(): #TODO
    print "SM : point_at" 
    time.sleep(3)
    return "succeeded"

# def call_follow(): #TODO
#   print "SM : follow" 
#   f = FollowMeSM()
#   out = f.execute()
#   return out


def call_find_object(object_name, loc_name):
    print "SM : find_object %s" % (object_name)
    
    # fo = SearchObjSM()
    # fo.userdata.object_to_search_for = object_name
    # out = fo.execute()
    # found_object = fo.userdata.object_found
    # return (out, found_object)
    # '''
    time.sleep(3)
    return "succeeded"


def call_grasp(obj):
    # print "SM : grasp %s" % (obj)
    # grasp = GraspSM()
    # grasp.userdata.object_to_search_for = obj
    # out = grasp.execute()
    # return out
    
    time.sleep(3)
    return "succeeded"

def call_find_person(person_name):
    print "SM : find_person %s" % (person_name)
    # fp = FindPersonSM()
    # out = fp.execute()
    # found_person = fo.userdata.closest_person
    # return out
    # '''
    time.sleep(3)
    return "succeeded"

def call_deliver(person_name): #Solo hace release TODO
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
    # return call_learn_person()
    # '''
    time.sleep(3)
    return "succeeded"

def call_introduce_me():
    print "SM : introduce_me" 
    # intr = IntroduceSM()
    # out = intr.execute()
    # return out
    # '''
    time.sleep(3)
    return "succeeded"


# def trad_obj(obj):
#   obj = int(obj)
#   if obj == 0:
#       return "coke"
#   else:
#       return "NULL"

# def trad_pers(pers):
#   pers = int(pers)
#   if pers==0:
#       return "albert"
#   else:
#       return "NULL"

# def trad_loc(loc):
#   loc = int(loc)
#   if loc == 0:
#       return "charger"
#   elif loc == 1:
#       return "kitchen"
#   elif loc == 2:
#       return "business"
#   elif loc == -1:
#       return "exit"
#   else:
#       return "NULL"

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
                    try: 
                        to_pers = command.GetParameterValue("pers")
                        # pers = trad_pers(to_pers)
                        pers = idx2obj(int(to_pers),'PERSONS')
                        print pers
                        if (pers =="NULL"):
                            print "ERROR: la persona %s no existe" % (to_pers)

                        out = call_deliver(pers)
                    except TypeError:
                        to_loc = command.GetParameterValue("loc")
                        # pers = trad_pers(to_pers)
                        loc = idx2obj(int(to_loc),'LOCATIONS')
                        print loc
                        if (loc =="NULL"):
                            print "ERROR: la localizacion %s no existe" % (to_loc)

                        out = call_deliver(loc)

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
                    '''pers_to_search = command.GetParameterValue("pers")
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
                    out = call_follow()

                elif command_name == "introduce-me":
                    out = call_introduce_me()

                elif command_name == "memorize-person":
                    out = call_learn_person()

                elif command_name == "recognize-person":
                    out = call_recognize_person()

                # elif command_name == "exit-apartment":
                #   loc_to_navigate = command.GetParameterValue("loc")
                #   loc = trad_loc(loc_to_navigate)
                #   print loc
                #   if (loc =="NULL"):
                #       print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)

                #   out = call_go_to(loc)

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
