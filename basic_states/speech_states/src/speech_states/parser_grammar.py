#! /usr/bin/env python
import string
import os
import rospkg

def parserGrammar(grammarFile):
    # We obtain all the values from grammarFile    
     
    tags = []
    rospack_instance = rospkg.RosPack()
    file_path = rospack_instance.get_path("speech_states")
    #pathFile = os.path.expanduser("~") + "/catkin_ws/src/robocup2014/reem_mocks/asr_mock/src/tags.txt"
    filePath = os.path.expanduser(file_path) + "/grammar/robocup/" + grammarFile + ".gram"
    
    # '/mnt_flash/etc/interaction/sphinx/model/gram/en_US/robocup/general.gram'
    # filePath = "/home/cristi/catkin_ws/src/robocup2014/basic_states/speech_states/grammar/" + grammarFile + '.gram'
     
    # Read the file
    f = open(filePath,'r')
    end = False
     
    # Read lines until we found "garbage"
    while not end:
        line = f.readline()

        # If has < in line[0] is a tag
        if line[0] == '<':
            # Obtain tag name 
            name, value = line.partition(">")[::2]
            print "Name : " + str(name)
            print "Value: " + str(value)
            tagObject = [name[1::]]

            if name[1::] == 'garbage':
                end = True
            else:   
                # Obtain different values
                value = value[string.find(value, '(')+1:string.find(value, ')'):]
                tagValues = value.strip(' ').split(' | ')                
                if str(value).count('"')==0 and str(value).count('$')==0:
                    print "Count: " + str(str(value).count('"'))
                    tagObject.append(tagValues)
                    tags.append(tagObject)
                    print "Value 2 : " + str(value)
                    print "tagValue 2 : " + str(tagValues) + "\n"
             
        # Else, next line
     
    # Close file
    f.close()
     
    return tags
     

def main():
    grammarName = 'deliver'
    tags = parserGrammar(grammarName)
    print tags

if __name__ == '__main__':
    main()
