from translator import obj2idx  #, idx2obj, get_list
import roslib
PERSON = 'PERSON'
ROBOT = 'ROBOT'
ITEM = 'ITEM'
DESIRE = 'DESIRE'

NO = 'no'
YES = 'yes'
ignore = 'ignore'

TARGET_FILE = "/SOAR/gp2/"


class goal():
    def __init__(self, oaction, oitem, operson, olocation):
        self.action = oaction
        self.person = operson
        self.item = oitem
        self.location = olocation

class item():
    def __init__(self):
        self.id = ignore
        self.locId = ignore
        self.found = ignore
        self.toBeGrasped = ignore
        self.grasped = ignore
        self.delivered = ignore
        self.pointed = ignore

class person():
    def __init__(self):
        self.id = ignore
        self.locId = ignore
        self.near = ignore
        self.obj1Id = ignore
        self.recognized = ignore
        self.memorized = ignore
        self.askedName = ignore
        self.followed = ignore
        self.found = ignore
        
class location():
    def __init__(self):
        self.id = ignore
        self.pointed_at = ignore

class robot():
    def __init__(self):
        self.id = '1'
        self.locId = ignore
        self.obj1Id = ignore
        self.introduced = ignore
        self.pointedAtLoc = ignore

class world():
    def __init__(self):
        self.item = item()
        self.person = person()
        self.robot = robot()
        # self.location = location()
        self.location = []
        self.location.append(location())
        self.name = ''

#     def new_object(self, Object, iters):
#             f = Object[0]
#             text = '   (<' + f + f + str(iters) + '> '
#             first_attr = True
#             ignore_field = True
#             obj = getattr(self, Object)
#             for n in obj.__dict__.keys():
#                 # print n
#                 if str(getattr(obj, n)) != ignore:
#                     if first_attr:
#                         text += '^' + n + ' ' + str(getattr(getattr(self, Object), n)) + '\n'
#                         first_attr = False
#                         ignore_field = False
#                     else:
#                         text += '           ^' first_attr+ n + ' ' + str(getattr(self.robot, n)) + '\n'
#             if ignore_field:
#                 text = ''
#             else:
#                 text += ')\n'
#             return text

    def new_world(self, allFile, locations):

        l = []
        lInits = []
        # locations = 
        for loc in locations:
            locc = obj2idx(loc, 'LOCATIONS')
            l.append(convert(locc))     #puts locc between <l > and adds it to the array l
            ll = '   (' + convert(locc) + ' ^id ' + str(locc) + ' ^pointed-at no)' #writes something like the following were 0 is locc
                                                                                   # (<l0> ^id 0 ^pointed-at no)
            lInits.append(ll)     #puts the previous phrase in to the array lInits
        allFile = allFile.replace('LOCATIONS', ' '.join(l)) #replaces LOCATIONS in allFile (witch is like  init-template.soar
        allFile = allFile.replace('   LOCATION', '\n'.join(lInits)) #the same as previous


        person_text = '   (<pers1> '    #builds the person and its atributes with the correct format to add it to the file
        first_attr = True
        ignore_field = True
        for n in self.person.__dict__.keys():
            if str(getattr(self.person, n)) != ignore:
                if first_attr:
                    person_text += '^' + n + ' ' + str(getattr(self.person, n))
                    first_attr = False
                    ignore_field = False
                else:
                    person_text += '\n           ^' + n + ' ' + str(getattr(self.person, n))
        if ignore_field:
            person_text = ''
        else:
            person_text += ')\n'

        robot_text = '   (<r> '
        first_attr = True
        ignore_field = True
        for n in self.robot.__dict__.keys():
            if str(getattr(self.robot, n)) != ignore:
                if first_attr:
                    robot_text += '^' + n + ' ' + str(getattr(self.robot, n))
                    first_attr = False
                    ignore_field = False
                else:
                    robot_text += '\n           ^' + n + ' ' + str(getattr(self.robot, n))
        if ignore_field:
            robot_text = ''
        else:
            robot_text += ')\n'

        item_text = '   (<obj1> '
        first_attr = True
        ignore_field = True
        for n in self.item.__dict__.keys():
            if str(getattr(self.item, n)) != ignore:
                if first_attr:
                    item_text += '^' + n + ' ' + str(getattr(self.item, n))
                    first_attr = False
                    ignore_field = False
                else:
                    item_text += '\n           ^' + n + ' ' + str(getattr(self.item, n))
        if ignore_field:
            item_text = ''
        else:
            item_text += ')\n'

        allFile = allFile.replace(ROBOT, robot_text)
        allFile = allFile.replace(ITEM, item_text)
        allFile = allFile.replace(PERSON, person_text)

        return allFile

    def update_world(self,GOAL):
        a=1
        print a
        for element in GOAL.__dict__.keys():
            if (element != 'name' and element != 'recognized'):
                ignore_field = True
                print GOAL
                print self
                print GOAL.__dict__.keys()
                print self.__dict__.keys()
                w_element = getattr(self, element)
                g_element = getattr(GOAL, element)
                try:
                    for n in g_element.__dict__.keys():
                        value = str(getattr(g_element, n))
                        if value != ignore:
                            setattr(w_element, n, value)
                            ignore_field = False
                except AttributeError:
                    idx = 0
                    for loc in g_element:
                        print loc
                        print g_element
                        gloc = g_element[0]
                        value = str(getattr(gloc, 'pointed_at'))
                        print 'heyya'
                        if value != ignore:
                            idx = int(getattr(gloc, 'id'))
                            wloc = w_element[idx]
                            setattr(wloc, 'pointed_at', value)
                            ignore_field = False
                            w_element[idx] = wloc
                        idx += 1
                if not ignore_field:
                    setattr(self, element, w_element)

        return self


    def get_goal(self, GOAL):
        verb = GOAL.action
        self.name = verb.replace('_', '-')
        if verb == 'go_to':
            self.goal_go_to(GOAL)
        elif verb == 'introduce':
            self.goal_introduce(GOAL)
        elif verb == 'follow':
            self.goal_follow(GOAL)
        elif verb == 'find' or verb == 'find_object' or verb == 'find_person':
            self.goal_find(GOAL)
        elif verb == 'grasp':
            self.goal_grasp(GOAL)
        elif verb == 'bring_to':
            self.goal_bring_to(GOAL)
        elif verb == 'bring_from':
            self.goal_bring_from(GOAL)
        elif verb == 'learn_person':
            self.goal_learn_person(GOAL)
        elif verb == 'exit':
            GOAL.location = obj2idx('exit', 'LOCATIONS')
            self.goal_go_to(GOAL)
        elif verb == 'recognize_person':
            self.goal_recognize(GOAL)
        elif verb == 'point_at':
            self.goal_point_at(GOAL)
        # return self.gen_goal()
        return self

    def gen_goal(self):
        ffirst_attr = True
        desi = '(<d> ^name ' + self.name
        ffirst_attr = False

        person_text = '   (<pp> '
        first_attr = True
        ignore_field = True
        for n in self.person.__dict__.keys():
            if str(getattr(self.person, n)) != ignore:
                if first_attr:
                    person_text += '^' + n + ' ' + str(getattr(self.person, n))
                    first_attr = False
                    ignore_field = False
                else:
                    person_text += '\n           ^' + n + ' ' + str(getattr(self.person, n))
        if ignore_field:
            person_text = ''
        else:
            if ffirst_attr:
                ffirst_attr = False
                desi += '^person <pp>'
            else:
                desi += '\n        ^person <pp>'
            person_text += ')\n'

        item_text = '   (<ii> '
        first_attr = True
        ignore_field = True
        for n in self.item.__dict__.keys():
            if str(getattr(self.item, n)) != ignore:
                if first_attr:
                    item_text += '^' + n + ' ' + str(getattr(self.item, n)) + '\n'
                    first_attr = False
                    ignore_field = False
                else:
                    item_text += '         ^' + n + ' ' + str(getattr(self.item, n))
        if ignore_field:
            item_text = ''
        else:
            if ffirst_attr:
                ffirst_attr = False
                desi += '^object <ii>'
            else:
                desi += '\n        ^object <ii>'
            item_text += ')\n'
        
        loc_text = '   (<ll> '
        first_attr = True
        ignore_field = True
        for n in self.location[0].__dict__.keys():
            if str(getattr(self.location[0], n)) != ignore:
                if first_attr:
                    loc_text += '^' + n + ' ' + str(getattr(self.location[0], n)) + '\n'
                    first_attr = False
                    ignore_field = False
                else:
                    loc_text += '           ^' + n + ' ' + str(getattr(self.location[0], n)) + '\n'
        if ignore_field:
            loc_text = ''
        else:
            if ffirst_attr:
                ffirst_attr = False
                desi += '^location <ll>'
            else:
                desi += '\n        ^location <ll>'
            loc_text += ')\n'

        
        # i = 1
        # obj = 'robot'
        # t=self.new_object(obj, i)
        # if t != '':
        #     if ffirst_attr:
        #         ffirst_attr = False
        #         desi += '^robot <' + obj[0] + obj[0] + str(i) + '>'
        #     else: 
        #         desi += '\n        ^robot <' + obj[0] + obj[0] + str(i) + '>'



        robot_text = '   (<rr> '
        first_attr = True
        ignore_field = True
        for n in self.robot.__dict__.keys():
            # print n
            if str(getattr(self.robot, n)) != ignore:
                if first_attr:
                    robot_text += '^' + n + ' ' + str(getattr(self.robot, n))
                    first_attr = False
                    ignore_field = False
                else:
                    robot_text += '\n         ^' + n + ' ' + str(getattr(self.robot, n)) 
        if ignore_field:
            robot_text = ''
        else:
            if ffirst_attr:
                ffirst_attr = False
                desi += '^robot <rr>'
            else:
                desi += '\n        ^robot <rr>'
            robot_text += ')\n'

        desi += ')\n'

        DesiredState = desi + person_text + item_text + loc_text + robot_text
        print DesiredState + '\n\n'
        return DesiredState

    def goal_bring_to(self, GOAL):
        if GOAL.person != '':
            self.person.id = GOAL.person
            self.person.obj1Id = GOAL.item
        elif GOAL.location != '':
            self.item.locId = GOAL.location
            print self.item.locId
        self.item.id = GOAL.item
        self.item.delivered = YES

    def goal_bring_from(self, GOAL):
        if GOAL.person != '':
            self.person.id = GOAL.person
            self.person.obj1Id = GOAL.item
        self.item.id = GOAL.item
        self.item.delivered = YES

    def goal_go_to(self, GOAL):
        self.robot.locId = GOAL.location

    def goal_find(self, GOAL):
        if GOAL.item != '':
            self.item.id = GOAL.item
            self.item.found = YES
        else:
            if GOAL.person != '':
                self.person.id = GOAL.person
                print self.person.id
                print 'just a sobre'
            else:
                print 'hauria de ser aqui a sobre'
                self.person.id = '-1'
            self.person.found = YES
            self.recognized = YES # ----------------------------------------------------------

    def goal_introduce(self, GOAL):
        self.robot.introduced = YES

    def goal_follow(self, GOAL):
        if GOAL.person != '':
            self.person.id = GOAL.person
        else:
            self.person.id = '-1'
        self.person.near = YES
        self.person.followed = YES

    def goal_learn_person(self, GOAL):
        if GOAL.person != '':
            self.person.id = GOAL.person
        else:
            self.person.id = '-1'
        self.person.memorized = YES

    def goal_point_at(self, GOAL):
        self.robot.pointedAtLoc = GOAL.location
        self.location[0].id = GOAL.location
        self.location[0].pointed_at = YES

    def goal_recognize(self, GOAL):
        if GOAL.person != '':
            self.person.id = GOAL.person
        else:
            self.person.id = '-1'
        self.person.recognized = YES
        self.person.near = YES

    def goal_grasp(self, GOAL):
        self.item.id = GOAL.item
        self.item.grasped = YES
        self.item.toBeGrasped = NO
        self.robot.obj1Id = GOAL.item

# def printNewGoal(oaction='go_to', oitem=0, operson=0, olocation='kitchen',
#     templatefile='-goal.soar', goalfile='goal-test'):
#     oaction = oaction.replace('_', '-')
#     templatesfilepath = roslib.packages.get_pkg_dir("gpsr") + "/src/goalSoars/"
#     targetfilepath = roslib.packages.get_pkg_dir("gpsrSoar") + "/SOAR/gp/elaborations/"
#     template = templatesfilepath + oaction + templatefile
#     target = targetfilepath + goalfile + '.soar'
#     tempfile = open(template, 'r')
#     targfile = open(target, 'w')
# 
#     for line in tempfile.readlines():
#         # print line
#         line = line.replace('ITEM', str(oitem))
#         line = line.replace('PERSON', str(operson))
#         line = line.replace('LOCATION', str(olocation))
#         targfile.write(line)
#     line = '\n\n'
#     targfile.write(line)
#     tempfile.close()
#     template = templatesfilepath + 'goalPart2.soar'
#     tempfile = open(template, 'r')
# 
#     for line in tempfile.readlines():
#         targfile.write(line)
# 
#     tempfile.close()
#     targfile.close()

def convert(element):
    el = '<l' + str(element) + '>'
    return el

def compileInit(oaction='go_to', oitem=0, operson=0, olocation=0,
    templatefile='init-template.soar', goalfile='initialize-gp.soar', locations=0, persons=0, items=0, current_world=world()):
    # given that all elements are passed as identifiers, only the number of elements must be set

    # oaction = oaction.replace('_', '-')
    templatesfilepath = roslib.packages.get_pkg_dir("gpsr") + "/src/goalSoars/"
    targetfilepath = roslib.packages.get_pkg_dir("gpsrSoar") + TARGET_FILE #"/SOAR/gp2/"
    template = templatesfilepath + templatefile
    target = targetfilepath + goalfile
    tempfile = open(template, 'r')
    targfile = open(target, 'w')

    allFile = tempfile.read()
    # print 'u'

    GOAL = goal(oaction, oitem, operson, olocation)

    print current_world.robot.locId
    allFile = current_world.new_world(allFile=allFile, locations=locations)

    # generate locations in world
    # l = []
    # lInits = []
    # # locations = 
    # for loc in locations:
    #     locc = obj2idx(loc, 'LOCATIONS')
    #     l.append(convert(locc))
    #     ll = '   (' + convert(locc) + ' ^id ' + str(locc) + ' ^pointed-at yes -)'
    #     lInits.append(ll)
    # allFile = allFile.replace('LOCATIONS', ' '.join(l))
    # allFile = allFile.replace('   LOCATION', '\n'.join(lInits))
    # robotInit = '(<r> ^locId -1\n        ^id 1\n        ^obj1Id -1\n        ^introduced yes -\n        ^pointed-at-loc <id> -)'
    # allFile = allFile.replace(ROBOT, robotInit)

    # PersonInit = []
    # if persons > 0:
    #     for per in persons:
    #         perss = obj2idx(per, 'PERSONS')
    #         p = '(<pers1> ^id ' + str(perss)
    #         PersonInit.append(p)
    #         # functions to be defined
    #         # get_pos must return '<ploc' + per + '> -' if not found
    #         # ploc = get_pos(translate(per, PERSON))
    #         p = '           ^locId <U2>'
    #         PersonInit.append(p)
    #         p = '           ^near yes -'
    #         PersonInit.append(p)
    #         p = '           ^obj1Id -1'
    #         PersonInit.append(p)
    #         p = '           ^recognized yes -'
    #         PersonInit.append(p)
    #         p = '           ^memorized yes -'
    #         PersonInit.append(p)
    #         p = '           ^askedName yes -'
    #         PersonInit.append(p)
    #         p = '           ^followed yes -)'
    #         PersonInit.append(p)
    # else:
    #     PersonInit = ''
    #     allFile = allFile.replace('\n        ^person <pers1>', '')
    # PInit = '\n'.join(PersonInit)
    # allFile = allFile.replace(PERSON, PInit)

    # print allFile

    # ItemInit = []
    # if items > 0:
    #     for it in items:
    #         itt = obj2idx(it, 'ITEMS')
    #         p = '   (<obj1> ^id ' + str(itt)
    #         ItemInit.append(p)
    #         # functions to be defined
    #         # get_pos must return '<ploc' + per + '> -' if not found
    #         # ploc = get_pos(idx2obj(it, ITEMS))
    #         # p = '           ^locId ' + ploc
    #         p = '           ^locId <U1>'
    #         ItemInit.append(p)
    #         p = '           ^found yes -'
    #         ItemInit.append(p)
    #         p = '           ^toBeGrasped yes'
    #         ItemInit.append(p)
    #         p = '           ^grasped yes -'
    #         ItemInit.append(p)
    #         p = '           ^delivered yes -'
    #         ItemInit.append(p)
    #         p = '           ^pointed yes -)'
    #         ItemInit.append(p)
    # else:
    #     ItemInit = ''
    #     allFile = allFile.replace('\n        ^object <obj1>', '')
    # IInit = '\n'.join(ItemInit)
    # allFile = allFile.replace(ITEM, IInit)

    desired_world = world()
    
    desired_world = desired_world.get_goal(GOAL)
    DesireInit = desired_world.gen_goal()
    allFile = allFile.replace(DESIRE, DesireInit)

    targfile.write(allFile)     #writes the file initialize-gp.soar

    targfile.close()
    tempfile.close()

    return desired_world
