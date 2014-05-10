#! /usr/bin/env python
import string
import os
import rospkg
import sys

from speech_states.parser_grammar import parserGrammar
from util_states.colors import Colors

def checkDictionaryGrammar(grammarFile):
    tags = parserGrammar(grammarFile)
    
    #Loading the path and the file
    rospack_instance = rospkg.RosPack()
    file_path = rospack_instance.get_path("speech_states")
    filePath = os.path.expanduser(file_path) + "/grammar/full.dic"
    
    f = open(filePath, 'r')
    end = False
    dic = {}
    while not end:
        line = f.readline()
        value = line.partition('\t')[0]
        if(str(value).isalpha()):
            dic[value] = str(value)
        elif (str(value)[1:] == 'unk'):
            end = True
#     print "Tags from parser: " + str(tags[:])
    existsInDic = True
    notExistedArray = []
    
     
    for tagValueArray in tags:
        for tagValue in tagValueArray:
            for tagInValue in tagValue:
                tagSeparated = str(tagInValue).split(' ')
                for tagFinal in tagSeparated :
                    if tagFinal.isalpha(): 
                        tagFinal = str(tagFinal).lower()
                        if not str(tagFinal) in dic:
                            existsInDic = False
                            notExistedArray.append(tagFinal);
#                             print "TAG Value: " + str(tagFinal) + " NOT Exists!"
    if existsInDic:
        print Colors().GREEN + "Dictionary OK!" + Colors().NATIVE_COLOR
    else:
        print Colors().RED + "Words that not exist : " + str(notExistedArray) + Colors().NATIVE_COLOR
    f.close()
    return existsInDic
        
        
    
def main():
    if(len(sys.argv) > 1):
        grammarName = sys.argv[1] 
    else:
        grammarName = 'deliver'

    checkDictionaryGrammar(grammarName)

if __name__ == '__main__':
    main()