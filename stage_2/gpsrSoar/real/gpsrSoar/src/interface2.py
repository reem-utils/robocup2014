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
from sm_gpsr_orders import TEST, SKILLS, TIME_INIT
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, PoseWithCovarianceStamped

from object_states.object_detect_sm import object_detect_sm
from object_states.recognize_object import recognize_object
from follow_me.follow_learn import LearnPerson
from follow_me.follow_operator import FollowOperator
from navigation_states.nav_to_poi import nav_to_poi
from face_states.new_database_and_learn import learn_face #new_database_and_learn as learn_face
from face_states.recognize_face import recognize_face
from face_states.go_find_person import go_find_person as SearchPersonSM
#from search_person_in_poi import SearchPersonSM
from util_states.point_to_poi import point_to_poi
from object_grasping_states.place_object_sm import place_object_sm
from object_grasping_states.pick_object_sm import pick_object_sm

#edit your path in gpsrSoar/src/pathscript.py


'''
SKILLS TODO:

--go_to (poi)
grasp    (object)           --> grasping -- TO TEST
bring_to(person)            --> grasping -- TO TEST
bring_to_loc(poi)           --> grasping -- TO TEST --CONSULT PARAM WITH each loc high
find_object(object)         --> object detection --always return succeeded regardless it seeing something or not
        faliable    --> not found
find_person(person)        --> TO TEST
        faliable    --> not found
point_at(poi) 
ask_name()
follow(person)              --> follow me --TO ASK ROGER
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

ROOMS = rospy.get_param('/robocup_params/rooms')
TABLES = rospy.get_param('/robocup_params/loc_category/table')

class dummy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
    def execute(self, userdata):
        return 'succeeded'
        
class speaker(smach.StateMachine): 


    def __init__(self, text=None, wait = True):
        #Initialization of the SMACH State machine
        smach.StateMachine.__init__(self,outcomes=['succeeded', 'preempted', 'aborted'])
        
        self.wait = wait
        with self: 
        
            self.userdata.tts_wait_before_speaking=0
            self.userdata.tts_text=None
            self.userdata.tts_lang=None
            
            smach.StateMachine.add(
                        'SaySM',
                        text_to_say(text,wait = self.wait),    #comment and uncomment dumy to make the robot anounce what he is going to do
                        #dummy(),
                        transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'aborted'})


def call_go_to(loc_name,world):

    tosay = "I'm going to the "+str(loc_name)
    speak = speaker(tosay,wait=False)
    speak.execute() 
    rospy.logwarn('call_go_to '+ loc_name + ' from :')  
    #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
         
        out = 'aborted'
        tries = 0       
        while(out=='aborted' and tries<3):
            tries = tries+1
            if world.get_current_position() == loc_name:  
                out = 'succeeded'
            else:
                if loc_name == "exit":
                    sm = nav_to_poi(poi_name = "door_B")
                    out = sm.execute() 
                 
                sm = nav_to_poi(poi_name = loc_name)
                out = sm.execute()     
             
             
        if out=='aborted':
            tosay = "I can't reach the " + loc_name + ". The door is closed. I'm going to inform"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN REACHING ' + loc_name)
            time.sleep(SLEEP_TIME)
             
             
            sm = nav_to_poi(poi_name = 'referee')
            out = sm.execute()     
            tosay = "I can't reach the " + loc_name + ". The door is closed. The sentence is from category 3"
            speak = speaker(tosay)
            speak.execute() 
             
            return "aborted"
        else:
            tosay = "I arrived to the " + loc_name
            speak = speaker(tosay)
            speak.execute()
    #############################################################################
    world.set_current_position(loc_name)
    rospy.logwarn(world.get_current_position())
    time.sleep(SLEEP_TIME)  
    return "succeeded" 

def call_guide_to(loc_name,world):

    tosay = "Please follow me to the "+str(loc_name)
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_guide_to '+ loc_name)  
#     #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
         
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = nav_to_poi(poi_name = loc_name)
            sm.execute()
             
             
        if out=='aborted':
            tosay = "I can't reach the " + loc_name + ". The door is closed. The sentence was from category 3"
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

    tosay = "I'm going to learn " + pers
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_learn_person ' + pers)     
    #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1            
#             sm = learn_face(name_face='john', name_database='Soar')
#             sm.userdata.name_face = pers
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
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = recognize_face()
            sm.userdata.name = pers
            out = sm.execute()  
               
        if out =='succeeded':
            tosay = "I recognized you, " + pers
            speak = speaker(tosay)
            speak.execute()
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
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            sm = point_to_poi(loc_name)    #to finish, test and include
            out = sm.execute()    
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
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):       
            tries = tries+1
            
            sm = follow_learn()    #to finish, test and include
            sm.execute()             
            
            sm2 = follow_operator()    #to finish, test and include
            out = sm2.execute()    
            sm2.userdata.in_learn_person = sm.userdata.in_learn_person
            #follow me
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_find_object(object_name,world): #TODO 
    
    tosay = "I'm going to search for " + object_name
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_find_object '+object_name)  
    #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted' 
        tries = 0
        world.item.object_pose = PoseWithCovarianceStamped()
        
        current_position = world.get_current_position()    
        rospy.logwarn(current_position)
        rospy.logwarn(ROOMS)
        rospy.loginfo('-----------------------------------------')
        if current_position in ROOMS:
            rospy.loginfo('-----------------------------------------')
            room = rospy.get_param('/robocup_params/room/' + current_position)
            for table in room :
                rospy.loginfo('-----------------------------------------')
                if world.item.object_pose.header.frame_id!='':
                    rospy.logwarn("NO ESTEM BUSCAN PERK YA HEM TROBAT EL QUE VOLIEM")
                    break
                call_go_to(table.replace(" ","_"),world)        
                tries = 0
                
                while(world.item.object_pose.header.frame_id=='' and tries<3):   
                    sm = recognize_object()
                    sm.userdata.object_name = object_name[0].upper() + object_name[1:]                
                    out = sm.execute()#
                    names = sm.userdata.object_detected_name
                    poses = sm.userdata.object_position
                    rospy.loginfo('-----------------------------------------')
                    rospy.loginfo(names)
                    rospy.loginfo('-----------------------------------------')
                    rospy.loginfo(poses)
                    rospy.loginfo('-----------------------------------------')
                    tries = tries+1
                
                    i = 0
                    while i<len(names):                    
                        if names[i] == (object_name[0].upper() + object_name[1:]):
                            world.item.object_pose = poses[i]
                        i = i +1                        
        
        if current_position.replace("_"," ") in TABLES:       
            while(world.item.object_pose.header.frame_id=='' and tries<3):   
                sm = recognize_object()
                sm.userdata.object_name = object_name[0].upper() + object_name[1:]                
                out = sm.execute()
                names = sm.userdata.object_detected_name
                poses = sm.userdata.object_position
                rospy.loginfo('-----------------------------------------')
                rospy.loginfo(names)
                rospy.loginfo('-----------------------------------------')
                rospy.loginfo(poses)
                rospy.loginfo('-----------------------------------------')
                tries = tries+1
            
                i = 0
                while i<len(names):                    
                    if names[i] == (object_name[0].upper() + object_name[1:]):
                        world.item.object_pose = poses[i]
                    i = i +1
                             
        rospy.logwarn(world.item.object_pose)      
             
        if out=='aborted':
            tosay = "I couldn't find the " + object_name + " you asked for. It isn't here. I'm going to the referee to inform"
            speak = speaker(tosay)
            speak.execute()
            rospy.logwarn('FAIL IN FINDING ' + object_name)
            time.sleep(SLEEP_TIME)
            
            call_go_to('referee',world)
            tosay = "I couldn't find the " + object_name + " you asked for. It isn't there. This sentence is from category 3"
            speak = speaker(tosay)
            speak.execute() 
            
            return "aborted"
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_grasp(obj,world): #TODO #adding grasping

    tosay = "I'm going to grasp the " + obj
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_grasp '+obj)  
    rospy.logwarn(world.item.object_pose)      
    #############################################################################
    if SKILLS :      
        pose_object_to_grasp = PoseStamped()
        pose_object_to_grasp.header = world.item.object_pose.header
        pose_object_to_grasp.pose = world.item.object_pose.pose.pose
        
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):   
            sm = pick_object_sm(pose_object_to_grasp)  #if not workng, blame chang
            out = sm.execute()
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
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<1):       
            tries = tries+1
            sm = SearchPersonSM('person')#person_name
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
    tosay = "Found you"
    speak = speaker(tosay)
    speak.execute()
    time.sleep(SLEEP_TIME)
    return "succeeded"

def call_bring_to(person_name): #TODO #Adding realese and reread tosay with some responsible person

    if person_name == '':
        tosay = "I'm leaving this here, sorry but you asked for a person without name"
    else:
        tosay = person_name + " can you pick this up please?"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_bring_to '+person_name)  
    #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        out = 'aborted'
        tries = 0
        while(out=='aborted' and tries<3):      
            
            person_object_position = PoseStamped()
            person_object_position.header.frame_id = "base_link"
            person_object_position.pose.position.x = 0.25
            person_object_position.pose.position.z = 1.0
            person_object_position.pose.orientation.w = 1.0  
            
            sm = place_object_sm(person_object_position)
            out = sm.execute()
            
            tries = tries+1
            #realese here
    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded" 

def call_bring_to_loc(location_name): #TODO #Improve toSay, add realese and, may be add some human recognition to avoid throwing stuff to the ground

#     if location_name == '':
    tosay = "I'm leaving this here"
#     else:
#         tosay = "I took this item here as requested. Referee I know you are here, if no one else is going to pick this proably you will want to take it before I throw it to the floor, thanks"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn('call_bring_to_loc '+location_name)  
    #############################################################################
    if SKILLS :      
        if (time.time()-TIME_INIT) > 270:
            return "succeeded"
        
        #param_place_high_name = "/robocup_params/" + location_name.replace(" ","_") + "_heigh"
        param_place_high_name = "/robocup_params/place/" + location_name.replace(" ","_")
        
        loc_object_position = PoseStamped()  #PoseWithCovarianceStamped()
        loc_object_position.header.frame_id = "base_link"
        loc_object_position.pose.position.z = 1.25
        loc_object_position.pose.position.x = 0.25
        loc_object_position.pose.orientation.w = 1.0  
        
        current_position = world.get_current_position()    
        rospy.logwarn(current_position)
        rospy.logwarn(ROOMS)   
                             
        if current_position == "kitchen":
            loc_object_position.pose.position.z = rospy.get_param("/robocup_params/place/bar")[3]
            loc_object_position.pose.position.x = rospy.get_param("/robocup_params/place/bar")[2]
                
            call_go_to("bar",world)    
            m = place_object_sm(loc_object_position)   
            out = sm.execute()               
            
        if current_position == "living room":       
            loc_object_position.pose.position.z = rospy.get_param("/robocup_params/place/dinner_table")[3]
            loc_object_position.pose.position.x = rospy.get_param("/robocup_params/place/dinner_table")[2]
                
            call_go_to("dinner table",world)   
            m = place_object_sm(loc_object_position)   
            out = sm.execute()             
                   
        if current_position == "hallway":
            loc_object_position.pose.position.z = rospy.get_param("/robocup_params/place/hallway_table")[3]
            loc_object_position.pose.position.x = rospy.get_param("/robocup_params/place/hallway_table")[2]
                
            call_go_to("hallway table",world)     
            m = place_object_sm(loc_object_position)   
            out = sm.execute()             
                 
        if current_position == "bedroom":
            loc_object_position.pose.position.z = rospy.get_param("/robocup_params/place/bed")[3]
            loc_object_position.pose.position.x = rospy.get_param("/robocup_params/place/bed")[2]
                
            call_go_to("bed",world)   
            m = place_object_sm(loc_object_position)   
            out = sm.execute()  
             
        
        if current_position.replace("_"," ") in TABLES:      
            loc_object_position.pose.position.z = rospy.get_param("/robocup_params/place/"+current_position.replace("_"," "))[3]
            loc_object_position.pose.position.x = rospy.get_param("/robocup_params/place/"+current_position.replace("_"," "))[2]
                  
            sm = place_object_sm(loc_object_position)   
            out = sm.execute()          

    #############################################################################
    time.sleep(SLEEP_TIME)
    return "succeeded" 

def call_ask_name(): #TOMAKESURE this is what we need if we even need this (learn person maybe)

    tosay = "Excuse me, would you mind telling me your name?"
    speak = speaker(tosay)
    speak.execute()
    rospy.logwarn( 'call_ask_name')
    
    if (time.time()-TIME_INIT) > 270:
        return "succeeded"
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
                rospy.logwarn(world.get_current_position())
                rospy.logwarn(str(time.time()) + '   ' + str(TIME_INIT))
                if (time.time()-TIME_INIT) > 270:
                    tosay = "time out."
                    speak = speaker(tosay)
                    speak.execute()
                    call_go_to('referee',world)
                    return "succeeded"

                out = "NULL"
                if command_name == "navigate":
                    loc_to_navigate = command.GetParameterValue("loc")
                    loc = idx2obj(int(loc_to_navigate), 'LOCATIONS')
                    print loc
                    if (loc =="NULL"):
                        print "ERROR: la loacalizacion %s no existe" % (loc_to_navigate)
                    
                    out = call_go_to(loc,world)

                elif command_name == "grasp":
                    obj_to_grasp = command.GetParameterValue("obj")
                    obj = idx2obj(int(obj_to_grasp),'ITEMS')
                    print obj
                    if (obj =="NULL"):
                        print "ERROR: el objeto %s no existe" % (obj_to_grasp)
                    
                    out = call_grasp(obj,world)

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
                    
                    out = call_find_object(obj,world)
                
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
                    #call_go_to('referee', world)
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
