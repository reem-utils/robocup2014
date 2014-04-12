import stparser as par #uses standfor parser (not workin for us)
a = par.StanfordParser(8)
sentence = "Go to the kitchen take a bottle from the table and find me at the station"

class dependency():
    def __init__(self):
        self.relation = ''
        self.mainVal = ''
        self.mainDepth = ''
        self.compVal = ''
        self.compDepth = ''

    def set_dependency(self, relation, mainval, maindepth, compval, compdepth):
        self.relation = relation
        self.mainVal = mainval
        self.mainDepth = maindepth
        self.compVal = compval
        self.compDepth = compdepth
        return self


class dependencies(list):
    def __init__(self):
        self = []

    def add_dependency(self, dep):
        self.append(dep)


class complement():
    def __init__(self):
        self.relation = ''
        self.value = ''

    def extract_complement(self, dep):
        self.relation = dep.relation
        self.value = dep.compVal
        return self


class verb():
    def __init__(self):
        self.verb = ' '
        self.complements = []

    def set_verb(self, v):
        self.verb = v
        return self

    def add_complement(self, dep):
        comp = complement()
        self.complements.append(comp.extract_complement(dep))


class verbList(dict):
    def __init__(self):
        self = {}

    def add_newverb(self, newverb):
        v = verb()
        self[newverb] = v.set_verb(newverb)

    def add_verb(self, newverb):
        self[newverb.verb] = newverb

    def add_complement(self, verb, dep):
        self[verb].add_complement(dep)


class orders():
    def __init__(self):
        self.action = ''
        self.item = ''
        self.person = ''
        self.location = ''
        self.others = []

    def kind_of_object(self, x):
        dictionary = {
            'dobj': 'item',
            'prep_to': 'location',
            'iobj': 'person',
            'prep_from': 'location',
            'prep_at': 'location' """,
            'acomp': '',
            'advcl': '',
            'agent': '',
            'attr': '',
            'ccomp': '',
            'conj': '',
            'csubj': '',
            'csubjpass': '',
            'aux': '',
            'auxpass': '',
            'dep': '',
            'expl': '',
            'npadvmod': '',
            'nsubj': '',
            'nsubjpass': '',
            'partmod': '',
            'prep': '',
            'prt': '',
            'purpcl': '',
            'tmod': '',
            'xcomp': '',
            'xsubj': ''"""
            }
        try:
            return dictionary[x]
        except:
            return ''

    def define_order(self, action1, item1, person1, location1):
        self.action = action1
        self.item = item1
        self.location = location1
        self.person = person1

    def set_order(self, v):
        self.action = v.verb
        for com in v.complements:
            kind = self.kind_of_object(com.relation)
            print com.value
            if kind == 'item':
                self.item = com.value
            elif kind == 'location':
                self.location = com.value
            elif kind == 'person':
                self.person = com.value
            else:
                self.others.append(com.value)


class orderList():
    def __init__(self):
        self.actionSet = []
        self.confidence = 'True'

    def extract_dependencies(self, deps):
        # Extract de relations in a structure
        o_deps = dependencies()
        for dep in deps:
            n1 = dep.find('(')
            n2 = dep.find(', ')
            n3 = dep.find(')')
            relation = dep[:n1]
            main = dep[n1 + 1:n2].split('-')
            comp = dep[n2 + 2:n3].split('-')
            # print main
            # print comp
            o_dep = dependency()
            o_deps.add_dependency(o_dep.set_dependency(
                relation,
                main[0],
                main[1],
                comp[0],
                comp[1]))

        return o_deps

    def parseOrders(self, sentence):
        c = a.batch_parse(sentence)
        d = c.strip().split("\n")
        # d.index('')

        # Extract parse tree data from Parser
        tree = d[:d.index('')]
        tree = '\n'.join(tree)

        # Extract dependencies data from Parse
        depends = d[d.index('') + 1:]

        # Prepare tree to extract verbs (or any other data)
        t = tree.strip().split(' ')

        n = True
        while n:
            try:
                t.remove('')
            except:
                n = False

        tt = []
        for part in t:
            try:
                part = part.replace("\n", "")
            except:
                True
            tt.append(part)

        # Find the verbs (predicates) in the sentence
        VerbList = []
        verbs = True
        ttt = " ".join(tt)

        while verbs:
            try:
                verbPos = ttt.find('VB')
                1 / (verbPos + 1)
                VerbList.append(ttt[verbPos + 3:verbPos + ttt[verbPos:].find(')')])
                ttt = ttt[verbPos + 3:]
            except:
                verbs = False

        # Ignore determinants, conjunctions and clausal complements
        deps = self.extract_dependencies(depends)
        rmList = ['det', 'ccomp', 'conj', 'root']
        print len(deps)
        for dep in deps:
            if dep.relation in rmList:
                deps.remove(dep)

        # Add verb complements from dependencies list
        verbs = verbList()
        print len(VerbList)
        for v in VerbList:
            verbs.add_newverb(v)
            print len(deps)
            for dep in deps:
                if dep.mainVal == v:
                    verbs.add_complement(v, dep)
        print len(verbs)

        # Assigns each verb to an order
        for v in verbs:
            o = orders()
            o.set_order(verbs[v])
            self.actionSet.append(o)

        return self
