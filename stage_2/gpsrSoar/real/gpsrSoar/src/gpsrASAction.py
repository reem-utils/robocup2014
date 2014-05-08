#! /usr/bin/env python

import roslib; roslib.load_manifest('gpsrSoar')
import rospy
import interface2 as interface
# from grammarReader import grammarFileReader as GFR
from grammarReader import grammarFileWriter2 as GFR
from GenerateGoalScript import world, person, location, item, robot, compileInit
from GenerateGoalScript import NO, YES, ignore
from translator import obj2idx, get_list, get_obj_location, idx2obj 

from speech_states.listen_general_command import askMissingInfo as askMissingInfoSM 
from speech_states.listen_general_command import askCategory as askCategorySM
from speech_states.listen_general_command import askCategoryLoc as askCategoryLocSM 

from sm_gpsr_orders import TEST

import actionlib
import gpsrSoar.msg
grammarNames = {}
grammarNames['persons'] = 'name'
grammarNames['locations'] = 'location'
grammarNames['items'] = 'item'
grammarNames['categories'] = 'category'
try:
  per = GFR(wordset=grammarNames)
except IOError:
  PATH = roslib.packages.get_pkg_dir("speech_states") + "/grammar/robocup/general.gram"
  #PATH = roslib.packages.get_pkg_dir("gpsrSoar") + "/src/general.gram"#"/src/gentest.gram"
  per = GFR(path=PATH, wordset=grammarNames)
print per

categories = ['drink', 'snack', 'cleaning_stuff', 'food', 'kitchenware']
loc_categories = ['door', 'table', 'shelf', 'appliance', 'seat', 'seating', 'utensil']
# except:
#   persons = {'homer': 0, '': ''}

# print 'blebla'
def indexGrammar(grammar):
  print grammar
  d = {}
  d = d.fromkeys(grammar) #.values()[0])
  i = 0
  for k in d.iterkeys():
    d[k] = i
    i += 1
  return d

print per

persons = indexGrammar(per['persons'])
# persons['referee'] = len(persons) + 1
# persons['albert'] = len(persons) + 1
# persons['jordi'] = len(persons) + 1
# persons['ricardo'] = len(persons) + 1
locations = indexGrammar(per['locations'])
items = indexGrammar(per['items'])
# items = {'coke': 0, 'apartment': 3, '': ''}
# locations = {'charger': 0, 'kitchen': 1, 'business': 2, 'apartment': 3, '' : ''}

# locations['exit'] = len(locations) + 1
persons[''] = ''
items['']=''
locations['']=''

def new_world(loc_list):  #generates a new world
  p = person()      #generates a person, names it referee and ask the translator about its localtion

  p.id = str(obj2idx('referee', 'PERSONS'))
  p.locId = str(obj2idx('referee', 'LOCATIONS')) # 2'
  p.near = NO
  p.obj1Id = '-1'
  p.recognized = NO
  p.memorized = NO
  p.askedName = NO
  p.followed = NO
  p.found = NO
  # check! if person = referee, location = robot location

  i = item()    #initializes items in the new world

  i.id = '-1'
  i.locId =  '-1' #str(obj2idx('sams_table', 'LOCATIONS')) # '-1'
  i.found = NO
  i.toBeGrasped = NO
  i.grasped = NO
  i.delivered = NO
  i.pointed = NO


  r = robot() # puts the robot himself in its world map

  r.id = '1'
  r.locId = str(obj2idx('referee', 'LOCATIONS'))
  r.obj1Id = '-1'
  r.introduced = NO
  r.pointedAtLoc = '-1' 
  print r.locId


  l = []
  for loc in loc_list: #adds the locations list to this world
    nl = location()

    nl.id = loc
    nl.pointed_at = NO
    l.append(nl)


  w = world() #generates a world and fill it with the objects we already created before.

  w.item = i
  w.person = p
  w.robot = r
  w.location = l

  return w  #return this world

def ask_data(Type='LOCATIONS', objectName='coke'):    
    if TEST:
        return 'fridge'
    ad = askMissingInfoSM(Type=Type, objectName=objectName)
    #ad.userdata._data = {'dataType': Type, 'object_name':objectName}
    #ad.userdata.dataType = Type
    #ad.userdata.object_name = objectName
    out = ad.execute()
    loc = ad.userdata._data['location_name']
    print "EL loc ES AKET!!!!!!!!!!!!!!!!!!!!!!!!!!!ask_data!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"+loc
    return loc  #-------------------'''

def ask_category(category):
    if TEST:
        if category == "drink":
            return 'coke'
        if category == "food":
            return 'tomato sauce'
        if category == "snack":
            return 'chocolate'
        if category == "cleaning":
            return 'deodorant'
        if category == "kitchenware":
            return 'spoon'
    ad = askCategorySM(GRAMMAR_NAME = category)
    out = ad.execute()
    obj = ad.userdata._data['object_name']
    #ob = obj2idx(ad.userdata.object_name, 'ITEMS')
    print "EL OBJ ES AKET!!!!!!!!!!!!!!!!!!!!!!!!!!!ask_category!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"+obj
    return obj   #--------------'''

def ask_category_loc(category):
    if TEST:
        if category == "table":
            return 'hallway table'
        if category == "shelf":
            return 'bar'
        if category == "appliance":
            return 'fridge'
        if category == "utensils":
            return 'plant'
        if category == "seat":
            return 'sofa'
        if category == "seating":
            return 'bench'
        if category == "door":
            return 'exit'
    ad = askCategoryLocSM(GRAMMAR_NAME = category) 
    out = ad.execute()
    obj = ad.userdata._data['location_name']
    #ob = obj2idx(ad.userdata.object_name, 'ITEMS')
    print "EL OBJ ES AKET!!!!!!!!!!!!!!!!!!!!!!!!!!!ask_category_loc!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"+obj
    return obj    #---------------------'''

def check_object_location(obj):
    if obj != '':
        # result = get_obj_location(obj)
        # if result == 'NULL':
        result = ask_data(Type='LOCATIONS', objectName=obj)
     
        return obj2idx(result, 'LOCATIONS')
    else:
        return '-1'
    #return 'sofa'

class gpsrASAction(object):
  _result   = gpsrSoar.msg.gpsrActionResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(name, gpsrSoar.msg.gpsrActionAction, execute_cb=self.execute_cb)
    self._as.start()
    self._hasgoal = False
    self._goalDonei = 0
    self._goal = []
    self._world = world()
    
    if TEST:
        errorfilepath = roslib.packages.get_pkg_dir("gpsr") + "/config/"
        error = errorfilepath + "error.txt"
        targfile = open(error, 'w')
        
  def execute_cb(self, goal):
    self._goalDonei = 0  #contador de goals

    self._goal = goal #saving our goal in self for more comfortable usage
    success = False  #flag to know when we have succeed   
    self._world = new_world(range(len(get_list('LOCATIONS'))))  #generates a new world
    print self._world.robot.locId
    self.print_goal()
    
    #"-----------------------------------------------------------orders list------------------------------"
    rospy.logwarn (str(self._goal.orderList))
    while not success:
      soarResult = interface.main()
      if soarResult == 'aborted':
        self._result.outcome = 'aborted'
        self._as.set_succeeded(self._result)
        success = True
        
        if TEST:
            rospy.logerr("THE SENTENCE IS: "+str(self._goal.orderList)+"\n")
            targfile.write(str(self._goal.orderList)+"\n")
      else: 
        if len(self._goal.orderList) == self._goalDonei:
          self._result.outcome = 'succeeded'
          rospy.loginfo('%s: Succeeded' % self._action_name)
          self._as.set_succeeded(self._result)
          success = True
        else:
          self._world.update_world(self._last_goal)

          self.print_goal()
  
    """
      for i in xrange(1, goal.order):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
    """

  def print_goal(self):
    commands = self._goal.orderList
    iperson = 0
    iitem = 0
    iloc = 0
    perss = []
    locc = []
    itt = []
    for c in commands:      #extract elements from orderList and put them in the arrays previously initialized
      if c.person != '':
        if c.person not in perss:
          iperson += 1
          perss.append(c.person)
      if c.item != '':
        if c.item not in itt:
          iitem += 1
          itt.append(c.item)
      if c.location != '':
        if c.location not in locc:
          iloc += 1
          locc.append(c.location)

    command = commands[self._goalDonei] #Extract command indexed by goalDonei to command

    locc = get_list('LOCATIONS') #Saves in locc all possible locations
    
    #this is a dummy for tests!!!
    # command = commands[1]
    # print command.action 

    # print command.location
    # print command.item
    # print command.person

    # printNewGoal(oaction=command.action, oitem=items[command.item], operson=persons[command.person], olocation=locations[command.location])
    # printNewGoal(oaction='go_to', oitem=items[command.item], operson=persons[command.person], olocation=1)
    # compileInit(locations=iloc, persons=iperson, items=iitem, oaction=command.action, oitem=items[command.item], operson=persons[command.person], olocation=locations[command.location])

    # checks that the item is or not a category
    # print 'blablabal'
    if command.item in categories: #if the item is a category this ask for further information and updates it
      objct = ask_category(command.item)
      self._world.item.id = obj2idx(objct, 'ITEMS')
      command.item = objct


    if command.location in loc_categories: #if the location is a category this ask for further information and updates it
      loca = ask_category_loc(command.location)
      command.location = loca

    # checks if the location of persons are known
    if self._world.person.locId == '-1' : # and command.person == self._world.person.id:
      ploc = check_object_location(idx2obj(int(self._world.person.id), 'PERSONS'))
      # if ploc != '-1':
      #   ploc = obj2idx(ploc,'LOCATIONS')
      print "LA LOCATION DE LA PERSON ES!!!!!!!!!!!!!!!!: "+str(ploc)
      self._world.person.locId = ploc
      
    
    try:    #adds to world the item id
      i = obj2idx(command.item, 'ITEMS')
      self._world.item.id = str(i)
    except ValueError:
      print str(self._world.item.id)
      print command.item
      i = ''
    try:    #adds to world the person id
      p = obj2idx(command.person, 'PERSONS')
      self._world.person.id = str(p)
    except ValueError:
      p = ''
    try:    
      l = obj2idx(command.location, 'LOCATIONS')
    except ValueError:
      l = ''
    print self._world.robot.locId
      
    
    if command.action == 'bring_from':
      self._world.item.locId = str(obj2idx(command.location, 'LOCATIONS'))

    if self._world.item.locId == '-1' and self._world.item.id != '-1': # If we know the item but not the location we check it and save it in world
      iloc = check_object_location(idx2obj(int(self._world.item.id), 'ITEMS'))
      self._world.item.locId = iloc

    if (command.action == 'memorize' or command.action == 'recognize'):
      command.person = self._world.person.id


    self._last_goal = compileInit(locations=locc, persons=perss, items=itt, oaction=command.action, 
                                  oitem=i, operson=p, olocation=l, current_world=self._world)
    '''
    locc: array with all the locations
    perss: array with all the persons involved in the commands
    itt: array with all the items involved in the commands
    comand.action: the action we want to perform
    i: the index of the object involved in this action
    p: index of the person involved in this action
    l: index of the location involved in this action
    self._world: the world we are working on
    '''

    self._goalDonei += 1 

  def has_succeeded(self):
    self._hasgoal = False
    if len(self._goal.orderList) == self._goalDonei:
      self._result.order_id  = goal.goalDonei
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
    print_goal()
    self._hasgoal = True

  def has_goal(self):
    return self._hasgoal


if __name__ == '__main__':
  rospy.init_node('gpsrSoar')
  gpsrASAction(rospy.get_name())
  rospy.spin()
