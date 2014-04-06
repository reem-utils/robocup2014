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
  PATH = roslib.packages.get_pkg_dir("gpsrSoar") + "/src/gentest.gram"
  print "si surt aixo al executar-se sobre REEM, es que s'ha de revisar que algo esta incorrecte a gpsrASAction.py line: 21"
  per = GFR(path=PATH, wordset=grammarNames)
print per

categories = ['drink', 'snack', 'cleaning_stuff', 'food']
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

def new_world(loc_list):
  p = person()

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

  i = item()

  i.id = '-1'
  i.locId =  '-1' #str(obj2idx('sams_table', 'LOCATIONS')) # '-1'
  i.found = NO
  i.toBeGrasped = NO
  i.grasped = NO
  i.delivered = NO
  i.pointed = NO


  r = robot()

  r.id = '1'
  r.locId = str(obj2idx('referee', 'LOCATIONS'))
  r.obj1Id = '-1'
  r.introduced = NO
  r.pointedAtLoc = '-1' 
  print r.locId


  l = []
  for loc in loc_list:
    nl = location()

    nl.id = loc
    nl.pointed_at = NO
    l.append(nl)


  w = world()

  w.item = i
  w.person = p
  w.robot = r
  w.location = l
  print w.robot.locId

  return w

def ask_data(Type='LOCATIONS', objectName='coke'):
 ad = askMissingInfoSM(Type=Type, objectName=objectName)
 #ad.userdata._data = {'dataType': Type, 'object_name':objectName}
 #ad.userdata.dataType = Type
 #ad.userdata.object_name = objectName
 out = ad.execute()
 loc = ad.userdata._data['location_name']
 return loc  #-------------------'''
 #return 'fridge'
  
 

def ask_category(category):
 ad = askCategorySM(GRAMMAR_NAME = category)
 out = ad.execute()
 obj = ad.userdata._data['object_name']
 ob = obj2idx(ad.userdata.object_name, 'ITEMS')
 return obj   #--------------'''
 #return 'milk'

def ask_category_loc(category):
 '''ad = askCategoryLocSM(GRAMMAR_NAME = category)
# ad.userdata._data = {'cat': category}
 #ad.userdata.cat = category
 out = ad.execute()
 print str(type(ad.userdata))
 obj = ad.userdata._data['loc_name']
 print obj
 # ob = obj2idx(ad.userdata.object_name, 'ITEMS')
 return obj    #---------------------'''
 return 'coke'

def check_object_location(obj):
  if obj != '':
      # result = get_obj_location(obj)
      # if result == 'NULL':
      result = ask_data(Type='LOCATIONS', objectName=obj)

      return obj2idx(result, 'LOCATIONS')
  else:
      return '-1'

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
    
  def execute_cb(self, goal):
    self._goalDonei = 0

    self._goal = goal
    success = False
    self._world = new_world(range(len(get_list('LOCATIONS'))))
    print self._world.robot.locId
    self.print_goal()
    
    while not success:
      soarResult = interface.main()
      if soarResult == 'aborted':
        self._result.outcome = 'aborted'
        self._as.set_succeeded(self._result)
        success = True
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
    for c in commands:      #extract elements from orderList and put them in the following arrays
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

    command = commands[self._goalDonei]

    locc = get_list('LOCATIONS')
    
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
    if command.item in categories:
      objct = ask_category(command.item)
      self._world.item.id = obj2idx(objct, 'ITEMS')
      command.item = objct
      print idx2obj(self._world.item.id, 'ITEMS')

    # print command.location
    # print 'here'
    if command.location in loc_categories:
      # print 'ther should be a print here'
      loca = ask_category_loc(command.location)
      # print locaask_category
      # print loca
      # print self._world.location
      # print len(self._world.location)
      # self._world.location.id = obj2idx(loca, 'LOCATIONS')
      command.location = loca
      # print loca

    # checks if the location of objects or persons are known
    if self._world.person.locId == '-1' : # and command.person == self._world.person.id:
      ploc = check_object_location(idx2obj(int(self._world.person.id), 'PERSONS'))
      # if ploc != '-1':
      #   ploc = obj2idx(ploc,'LOCATIONS')
      self._world.person.locId = ploc
    
    # print command.item
    try:
      i = obj2idx(command.item, 'ITEMS')
      self._world.item.id = str(i)
    except ValueError:
      print str(self._world.item.id)
      print command.item
      i = ''
    try:
      p = obj2idx(command.person, 'PERSONS')
      self._world.person.id = str(p)
    except ValueError:
      p = ''
    try:
      l = obj2idx(command.location, 'LOCATIONS')
      # print l
      # print l
    except ValueError:
      l = ''
    print self._world.robot.locId
    # print command.location
    # print idx2obj(l, 'LOCATIONS')
    # print 'blabla'

    # print 'here here ' + str(p)

    if self._world.item.locId == '-1' and self._world.item.id != '-1': # and command.item == self._world.item.id:
      iloc = check_object_location(idx2obj(int(self._world.item.id), 'ITEMS'))
      # if iloc != '-1':
      #   iloc = obj2idx(iloc,'LOCATIONS')
      self._world.item.locId = iloc
    
    if command.action == 'bring_from':
      self._world.item.locId = str(obj2idx(command.location, 'LOCATIONS'))

    if (command.action == 'memorize' or command.action == 'recognize'):
      command.person = self._world.person.id

    # print self._world.item.id
    # print self._world.item.locId

    self._last_goal = compileInit(locations=locc, persons=perss, items=itt, oaction=command.action, oitem=i, operson=p, olocation=l, current_world=self._world)
    # compileInit(locations=locc, persons=perss, items=itt, oaction=command.action, oitem=items[command.item], operson=persons[command.person], olocation=locations[command.location])

    
    # printNewGoal(action, it, pers, loc, self._goalDonei)
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
