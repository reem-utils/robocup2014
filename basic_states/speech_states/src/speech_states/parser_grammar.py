#! /usr/bin/env python
import string

def parserGrammar(grammarFile):
    # We obtain all the values from grammarFile    
     
    tags = []
    filePath = "/home/cristi/catkin_ws/src/robocup2014/basic_states/speech_states/grammar/" + grammarFile
     
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
            tagObject = [name[1::]]

            if name[1::] == 'garbage':
                end = True
            else:   
                # Obtain different values
                value = value[string.find(value, '(')+1:string.find(value, ')'):]
                tagValues = value.strip(' ').split(' | ')
             
                tagObject.append(tagValues)
                tags.append(tagObject)
             
        # Else, next line
     
    # Close file
    f.close()
     
    return tags
     

def main():
    grammarName = 'yes_no.gram'
    tags = parserGrammar(grammarName)
    print tags

if __name__ == '__main__':
    main()
