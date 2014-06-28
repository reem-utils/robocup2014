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
from GenerateGoalScript import world
from speech_states.say import text_to_say
from sm_gpsr_orders import TEST, SKILLS
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

#TODO: search_object_with_confidence import SearchObjectWithConfidenceStateMachine as SearchObjSM
from navigation_states.nav_to_poi import nav_to_poi #navigation.move_to_room import MoveToRoomStateMachine as MoveToRoomSM
# from pal_smach_utils.navigation.follow_and_stop import FollowAndStop as FollowMeSM
from face_states.learn_face import learn_face
from face_states.recognize_face import recognize_face
from search_person_in_poi import SearchPersonSM
from util_states.point_to_poi import point_to_poi
from object_grasping_states.place_object_sm import place_object_sm
from object_grasping_states.pick_object_sm import pick_object_sm

#edit your path in gpsrSoar/src/pathscript.py

'''
SKILLS TODO:

--go_to (poi)
-grasp    (object)           --> grasping -- TO TEST
-bring_to(person)            --> grasping -- TO TEST
-bring_to_loc(poi)           --> grasping -- TO TEST --CONSULT PARAM WITH each loc high
find_object(object)         --> object detection --Faked
-        faliable    --> not found
--find_person(person)
--        faliable    --> not found
--point_at(poi) 
--ask_name()
follow(person)              --> follow me
--introduce_me()
--learn_person(person)        --> face recognition
--recognize_person(person)    --> face recognition

'''

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
if TEST:
    SLEEP_TIME = 0
else:
    SLEEP_TIME = 3

object_position = PoseStamped()

class dummy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
    def execute(self, userdata):
        return 'succeeded'
        
class speaker(smach.StateMachine): 


    def __init__(self, text=None):
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self,outcomes=['succeeded', 'preempted', 'aborted'])
        
        with self: 
        
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            
            smach.StateMachine.add(
                        'SaySM',
                        text_to_say(text),    #uncomment and comment dumy to make the robot anounce what he is going to do
                        #dummy(),
                        transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'aborted'})


def call_go_to(loc_name,world):

    tosay = "I'm going to the "+str(loc_name)
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_go_to '+ loc_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = nav_to_poi(poi_name = loc_name)
            out = sm.execute()     
            
            
        if out=='aborted':
            tosay = "I can't reach the " + loc_name + ". The door must be closed. I'm going to the referee to inform"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN REACHING ' + loc_name)
            time.sleep(SLEEP_TIME)
            
            call_go_to('referee',world)
            tosay = "I can't reach the " + loc_name + ". The door must be closed. I'm afraid that sentence was from category 3"
            speak = speaker(tosay)
            speak.execute() 
            
            return "aborted"
    #############################################################################
    world.set_current_position(loc_name)
    time.sleep(SLEEP_TIME)  
    return "succeeded" 

def call_guide_to(loc_name,world):

    tosay = "Please follow me to the "+str(loc_name)
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_guide_to '+ loc_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = nav_to_poi(poi_name = loc_name)
            sm.execute()
            
            
        if out=='aborted':
            tosay = "I can't reach the " + loc_name + ". The door must be closed. I'm afraid that sentence was from category 3"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN REACHING ' + loc_name)
            time.sleep(SLEEP_TIME)            
            return "aborted"
    #############################################################################
    world.set_current_position(loc_name)
    time.sleep(SLEEP_TIME)  
    return "succeeded" 

def call_learn_person(pers): #TOTEST   #Recorda que abans sempre busca una persona que encara no coneix, revisar SOAR

    tosay = "I'm going to learn the person in front of me, known as " + pers
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_learn_person ' + pers)    
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1            
            sm = learn_face()
            sm.userdata.name = pers
            out = sm.execute()     
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_recognize_person(pers): #TODO  PersonName maybe?

    tosay = "I'm going to recognize " + pers
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_recognize_person ' + pers)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = recognize_face()
            sm.userdata.name = pers
            out = sm.execute()     
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded" 

def call_point_at(loc_name): #TODO  #to finish, test and include
    
    tosay = "I'm going to point to " + loc_name
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_point_at ' + loc_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = point_to_poi(loc_name)    #to finish, test and include
            sm.execute()    
    #############################################################################    
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_follow(pers): #TODO   
    print pers
    rospy.logwarn("SM : follow-me")
    tosay = "I'm going to follow " + pers
    speak = speaker(tosay)
    speak.execute()
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            #follow me
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_find_object(object_name): #TODO 
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
    
    tosay = "I'm going to search for " + object_name
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_find_object '+object_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):      
            ###
            out = 'succeeded'
            object_position.header.frame_id = "base_link"
            object_position.pose.position.x = 0.5
            object_position.pose.position.z = 1.0
            object_position.pose.orientation.w = 1.0 
            ###
            tries = tries+1
            
            
        if out=='aborted':
            tosay = "I couldn't find the " + object_name + " you asked for. It isn't here. I'm going to the referee to inform"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN FINDING ' + object_name)
            time.sleep(SLEEP_TIME)
            
            call_go_to('referee',world)
            tosay = "I couldn't find the " + object_name + " you asked for. It isn't there. I'm afraid that sentence was from category 3"
            speak = speaker(tosay)
            speak.execute() 
            
            return "aborted"
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_grasp(obj): #TODO #adding grasping

    tosay = "I'm going to grasp the " + obj
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_grasp '+obj)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):   
            pick_object_sm(object_position)  #if not workng, blame chang
            tries = tries+1       
            #grasping here
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_find_person(person_name): #TOTEST

    tosay = "I'm going to search for the person known as " + person_name
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_find_person '+person_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<1):       
            tries = tries+1
            sm = SearchPersonSM(person_name)
            out = sm.execute()
            
            
        if out=='aborted':
            tosay = "I couldn't find the person known as " + person_name + ". He isn't here. I'm going to the referee to inform"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN FINDING '+person_name)
            time.sleep(SLEEP_TIME)
            
            call_go_to('referee',world)
            tosay = "I couldn't find the person known as " + person_name + ". He isn't there. I'm afraid that sentence was from category 3"
            speak = speaker(tosay)
            speak.execute() 
            
            return "aborted"
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_bring_to(person_name): #TODO #Adding realese and reread tosay with some responsible person

    if person_name == '':
        tosay = "I'm leaving this here, sorry but you asked for a person without name"
    else:
        tosay = person_name + "would you mind picking this up when I release it?"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_bring_to '+person_name)
    #############################################################################
    if SKILLS :
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):      
            
            person_object_position = PoseStamped()
            person_object_position.header.frame_id = "base_link"
            person_object_position.pose.position.x = 0.5
            person_object_position.pose.position.z = 1.25
            person_object_position.pose.orientation.w = 1.0  
            
            place_object_sm(person_object_position)
            
            tries = tries+1
            #realese here
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded" 

def call_bring_to_loc(location_name): #TODO #Improve toSay, add realese and, may be add some human recognition to avoid throwing stuff to the ground

    if location_name == '':
        tosay = "I'm leaving this here"
    else:
        tosay = "I took this item here as requested. Referee I know you are here, if no one else is going to pick this provably you will want to take it before I throw it to the floor, thanks"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_bring_to_loc '+location_name)    
    #############################################################################
    if SKILLS :
        param_name = "/robocup_params/" + location_name.replace(" ","_") + "_heigh"
        
        loc_object_position = PoseStamped()
        loc_object_position.header.frame_id = "base_link"
        loc_object_position.pose.position.x = 0.5
        if rospy.has_param(param_name):            
            loc_object_position.pose.position.z = rospy.get_param(param_name)
        else:
            loc_object_position.pose.position.z = 1.25
        loc_object_position.pose.orientation.w = 1.0  
    
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):              
            
            place_object_sm(loc_object_position)                   
            
            tries = tries+1
            #realese here
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded" 

def call_ask_name(): #TOMAKESURE this is what we need if we even need this (learn person maybe)

    tosay = "Excuse me, would you mind telling me your name?"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn( 'call_ask_name')
    #############################################################################
    '''
    Maybe we should save that the person in front of me is, instead of a random person the one with the identifier asociated to his name
    '''
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_introduce_me(): #TOASKSAM for a proper introduction
    
    tosay = "Hi, I am reem a robot designed by PAL robotics and prepared by la Salle students to take part in the robocup competition"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn( 'call_introduce_me')
    time.sleep(SLEEP_TIME)
    return "succeeded"


def define_prohibitions(): #TODISCOVER WTF IS THIS
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


def main(world):
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
    print str(res)
    

    goal_achieved = False
    while not goal_achieved:
        agent.Commit()  
        agent.RunSelfTilOutput()
        agent.Commands()
        numberCommands = agent.GetNumberCommands()
        print "Numero de comandos recibidos del agente: %s" % (numberCommands)
        i=0
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
                    
                    out = call_go_to(loc,world)

                elif command_name == "grasp":
                    obj_to_grasp = command.GetParameterValue("obj")
                    # obj = trad_obj(obj_to_grasp)
                    obj = idx2obj(int(obj_to_grasp),'ITEMS')
                    print obj
                    if (obj =="NULL"):
                        print "ERROR: el objeto %s no existe" % (obj_to_grasp)
                    
                    out = call_grasp(obj)

                elif command_name == "deliver": #to Person
                    try:
                        to_pers = command.GetParameterValue("pers")
                        pers = idx2obj(int(to_pers),'PERSONS')
                        print pers
                        if (pers =="NULL"):
                            print "ERROR: la persona %s no existe" % (to_pers)                        
                        out = call_bring_to(pers)
                    except:                     #or to Loc
                        to_loc = command.GetParameterValue("loc")
                        try:
                            loc = idx2obj(int(to_loc),'LOCATIONS')
                            print loc
                            if (loc =="NULL"):
                                print "ERROR: l'objecte %s no existe" % (to_loc)                        
                            out = call_bring_to_loc(loc)
                        except:
                            loc = ''   
                            print loc
                            out = call_bring_to_loc(loc)

                elif command_name == "search-object":
                
                    obj_to_search = command.GetParameterValue("obj")
                    obj = idx2obj(int(obj_to_search), 'ITEMS')
                    print obj
                    if (obj =="NULL"):
                        print "ERROR: el objeto %s no existe" % (obj_to_search)
                    
                    out = call_find_object(obj)
                
                elif command_name == "search-person":
                    pers_to_search = command.GetParameterValue("pers")
                    pers = idx2obj(int(pers_to_search), 'PERSONS')
                    print pers
                    if (pers =="NULL"):
                        print "ERROR: la persona %s no existe" % (pers_to_search)
                    
                    out = call_find_person(pers)
                

                elif command_name == "point-obj":                    
                    to_loc = command.GetParameterValue("loc")
                    loc = idx2obj(int(to_loc),'LOCATIONS')
                    out = call_point_at(loc)
                
                elif command_name == "ask-name":
                    out = call_ask_name()
                
                elif command_name == "follow":          
                    to_pers = command.GetParameterValue("pers")
                    pers = idx2obj(int(to_pers),'PERSONS')
                    out = call_follow(pers)
                
                elif command_name == "introduce-me":
                    out = call_introduce_me()
                
                elif command_name == "memorize-person":
                    to_pers = command.GetParameterValue("pers")
                    pers = idx2obj(int(to_pers),'PERSONS')
                    out = call_learn_person(pers)
                
                elif command_name == "recognize-person":
                    to_pers = command.GetParameterValue("pers")
                    pers = idx2obj(int(to_pers),'PERSONS')
                    out = call_recognize_person(pers)
                    
                elif command_name == "guide":
                    loc_to_navigate = command.GetParameterValue("loc")
                    loc = idx2obj(int(loc_to_navigate), 'LOCATIONS')
                    print loc
                    if (loc =="NULL"):
                        print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)
                    out = call_guide_to(loc,world)

                elif command_name == "achieved":
                    goal_achieved = True
                    out = "succeeded"
                
                else:
                    print "ERROR: El commando %s no existe" % (command_name)
                    command.AddStatusComplete()


                print "SM return: %s \n\n" % (out) 
                if out=="succeeded": 
                    command.AddStatusComplete()
                elif out=="aborted":
                    command.AddStatusError()
                else:
                    print "gpsrSoar interface: unknown ERROR"
                    exit(1)
                
                i+=1

    command.AddStatusComplete()
    return 'succeeded'


    kernel.DestroyAgent(agent)
    kernel.Shutdown()
    del kernelCommit


if __name__ == "__main__":
    main()
