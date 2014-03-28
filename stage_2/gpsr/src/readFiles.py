import roslib


class functions():
    def __init__(self, name):
        self.name = name
        self.verbs = []

    def addVerb(self, v):
        self.verbs.append(v)


class functionList(dict):

    def importFile(self):
        PATH = roslib.packages.get_pkg_dir("gpsr")
        File = PATH + 'src/VerbCategories.txt'
        for relation in [relations.strip('\n') for relations in open(File, 'r').readlines()]:
            relation = relation.split(' ')
            #found = False

            try:
                self[relation[1]].addVerb(relation[0])
            except:
                self[relation[1]] = functions(relation[1])
                self[relation[1]].addVerb(relation[0])
"""
            for item in self:
                if relation[1] == item.name:
                    item.addVerb(relation[0])
                    found = True
                else:
                    found = False
            if not found:
                self.append(functions(relation[1]))
                self[len(self) - 1].addVerb(relation[0])
"""


class inv_functionList(functionList):
    def __init__(self):
        PATH = roslib.packages.get_pkg_dir("gpsr")
        File = PATH + '/src/VerbCategories.txt'
        for relation in [relations.strip('\n') for relations in open(File, 'r').readlines()]:
            relation = relation.split(' ')
            self[relation[0]] = relation[1]
