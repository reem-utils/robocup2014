exceptions_list = ['exit']
exception_cb = {}
not_verb_relations = ['det', 'amod']
import nltk
from grammarReader import grammarFileReader as GFR
import roslib



def checkNames(sentence):
    nameList = GFR(wordset={'names': 'nameshort'})['names']
    for word in sentence.split(' '):
        if word in nameList[0]:
            word = word + '/NN'

def is_noun(relation):
    if relation in not_verb_relations:
        return True
    else:
        return False


def tokenize_sentence(sent):
    t_sent = nltk.word_tokenize(sent)
    return t_sent


def detokenize_sentence(sent):
    d_sent = ' '.join(sent)
    return d_sent


def exit_cb(index, n_sent, deps):
    for dep in deps:
        if dep.mainVal == n_sent[index] and is_noun(dep.relation):
            mod = '/NN'
        else:
            mod = '/VB'
    # print mod
    n_sent[index] = n_sent[index] + mod
    return n_sent


def check_sentence_changed(sent, n_sent):
    # print 'check!!'
    # print sent
    # print n_sent
    # print sent == n_sent
    if sent == n_sent:
        return False
    else:
        return True

# define here all exception callbacks
exception_cb['exit'] = exit_cb


def change_exceptions(sent, dependences):
    try:
        nameList = GFR(wordset={'names': 'nameshort'})['names']
    except IOError:
        PATH = roslib.packages.get_pkg_dir("gpsrSoar") + "/src/iam.gram"
        nameList = GFR(path=PATH, wordset={'names': 'nameshort'})['names']


    i = 0
    n_sent = sent[:]
    # print sent
    for word in sent:
        print word
        print nameList
        if word in nameList:
            word = word + '/NN'
            n_sent[i] = word
        try:
            n_sent = exception_cb[word](index=i, n_sent=n_sent, deps=dependences)
            i += 1
        except KeyError:
            i += 1
    # print n_sent
    # print sent
    return n_sent


def clean_word(word):
    print 'the word is'
    print word
    word.split('/')
    try:
        return word[0]
    except IndexError:
        return ''