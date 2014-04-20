import roslib
import string
from translator import get_list
def grammarFileReader(wordset, path='/mnt_flash/etc/interaction/sphinx/model/gram/en_US/robocup/general.gram'):
    # Reads a grammar until it finds the desired wordset
    # Returns an array of the words wich mach the key given in wordset
    # Should accept wordset as string or list of strings
    # a singe wordset should be passed as a list of one item
    f = open(path,'r')
    n = len(wordset)
    found = False
    wordlist = {}
    while not found:
        line = f.readline()
        for item in wordset.keys():
            it = '<' + wordset[item] + '> ='
            ind = string.find(line, it)
            if ind >= 0:


                data = line[string.find(line, '=') + 1:len(line)-2]
                data = data.strip(' () ') #.strip('(').strip(')').strip(' ')
                wordlist[item] = data.strip(' ').split(' | ')
        if line == '':
            found = True
        if len(wordlist) == n:
            found = True
    return wordlist


def grammarFileWriter(wordlist, path=''):
    print 'aqui entra?'
    if path == '':
        path = roslib.packages.get_pkg_dir('gpsr') + '/src/'
        # should be changed by a try-except statement

    f = open(path + 'newGrammar.cfg','w')
    ff = open(path + 'sampleGrammar.cfg', 'r')
    # l = ff.readline()
    # finished = False

    text = ff.read()
    text = text.replace('ITEM -> "ITEM"', 'ITEM -> "' + '" | "'.join(wordlist['items']) + '" ' )
    text = text.replace('LOCATION -> "LOCATION"', 'LOCATION -> "' + '" | "'.join(wordlist['locations']) + '" ' )
    text = text.replace('PERSON -> "PERSON"', 'PERSON -> "' + '" | "'.join(wordlist['persons']) + '" ' )
    f.write(text)

    # for line in ff.readlines():
    #     if line == 'ITEM -> "ITEM"':
    #         f.write(line)
    #         f.write('ITEM -> "' + '" | "'.join(wordlist['items']) + '" ')
    #     elif line == 'LOCATION -> "LOCATION"':
    #         f.write(line)
    #         f.write('LOCATION -> "' + '" | "'.join(wordlist['locations']) + '" ')
    #     elif line == 'PERSON -> "PERSON"':
    #         f.write(line)
    #         f.write('PERSON -> "' + '" | "'.join(wordlist['persons']) + '" ')
    #     else:
    #         f.write(line)

    f.close()
    ff.close()
    return 'ok'

def grammarFileWriter2(wordset, path='/mnt_flash/etc/interaction/sphinx/model/gram/en_US/robocup/general.gram'):
    # print 'aqui entra?'
    # if path == '':
    #     path = roslib.packages.get_pkg_dir('gpsrSoar') + '/src/gentest.gram'
    #     # should be changed by a try-except statement

    # ff = open(path, 'r')
    # # l = ff.readline()
    # # finished = False

    # text = ff.read()

    # ff.close()
    print wordset.keys()

    # n = len(wordset)
    found = True
    # fcheck = False
    wordlist = {}
    for item in wordset.keys():
        # if not found:
        #     found = True
        #     fcheck = True
        # it = '<' + wordset[item] + '> ='
        # st = string.find(text, it)
        # if st >= 0:
        List = get_list((wordset[item] + 's').upper())
        # to_replace = text[st:]
        # to_replace = to_replace[:string.find(to_replace, '\n')]
        # new_data = it + ' ( ' + ' | '.join(List) + ' );'
        # text = text.replace(to_replace, new_data)
        wordlist[item] = List
            # print 'found found'
        # else:
        #     found = False
        #     print 'Word Set not found. \n Please revise that '
        #     print '\n'.join(wordset[item])
        #     print 'are included in the grammar with this exact names. \nIf not, please change them'

    print found
    if not found:
        print 'Word Set not found. \n Please revise that '
        print '\n'.join(wordset.keys())
        print 'are included in the grammar with this exact names. \nIf not, please change them'

        return ''
    else: 
        # f = open(path,'w')
        # f.write(text)
        # f.close()
        return wordlist

    return 'ok'