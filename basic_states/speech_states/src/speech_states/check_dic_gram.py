#! /usr/bin/env python
import string
import os
import rospkg
import sys

from speech_states.parser_grammar import parserGrammar

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
        print "Line is: " + str(line)
        value = line.partition('\t')[0]
        print "Value: " + str(value)
        if(str(value).isalpha()):
            dic[value] = str(value)
        elif (str(value)[1:] == 'unk'):
            end = True
    print "Tags from parser: " + str(tags[:])
    existsInDic = True
    for tagValueArray in tags:
        for tagValue in tagValueArray:
            print "TAG Value :::: " + str(tagValue)
            for tagInValue in tagValue:
                tagSeparated = str(tagInValue).split(' ')
                for tagFinal in tagSeparated :
                    if tagFinal.isalpha(): 
                        if str(tagFinal) in dic:
                            print "TAG Value: " + str(tagFinal) + " Exists!"
                        else:
                            existsInDic = False
                            print "TAG Value: " + str(tagFinal) + " NOT Exists!"
    print "Exists??????? " + str(existsInDic)
    return existsInDic
        
        
    
def main():
    if(len(sys.argv) > 1):
        grammarName = sys.argv[1] 
    else:
        grammarName = 'deliver'
    
    print "MAAAIN"
    checkDictionaryGrammar(grammarName)

if __name__ == '__main__':
    main()