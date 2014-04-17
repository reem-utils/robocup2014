#######################################################
# General purpose test for the RoboCup@Home competition
# version: Singapore 2010
#
# Programmed by:
# Tijn van der Zant
# email: tijn@ieee.org
#
#######################################################

# imports
import random
import sys
import copy

# read the locations, objects and sentences files
# and clean up the lists from the files
locations       = []
items           = []
cat1Sentences   = []
cat2Sentences   = []
cat3Situations  = []
names           = []
# get rid of empty lines and do not use anything that starts with a '#'
for loc in [location.strip('\n') for location in open('locations.txt', 'r').readlines()]:
    if loc != '':
        if loc[0] != '#':
            locations.append(loc)
for it  in [item.strip('\n')     for item     in open('items.txt',   'r').readlines()]:
    if it  != '':
        if it[0] != '#':
            items.append(it)
for sentence in [str(sent).strip('\n') for sent in open('cat1Sentences.txt' , 'r').readlines()]:
    if sentence != '':
        if sentence[0] != '#':
            cat1Sentences.append(sentence)
for sentence in [str(sent).strip('\n') for sent in open('cat2Sentences.txt' , 'r').readlines()]:
    if sentence != '':
        if sentence[0] != '#':
            cat2Sentences.append(sentence)
situations  = []
questions   = []
for sit in [str(sent).strip('\n') for sent in open('cat3Situations.txt' , 'r').readlines()]:
    if sit != '':
        if sit[0] != '#':
            if sit.split()[0] == 'situation:':
                situations.append( sit )
            if sit.split()[0] == 'question:':
                questions.append( sit )
cat3Situations = zip( situations, questions )
for name in [nam.strip('\n')     for nam     in open('names.txt',   'r').readlines()]:
    if name  != '':
        if name[0] != '#':
            names.append(name)

# are there at least two locations?
if len(locations) < 2:
    print 'Not enough locations. Exiting program'
    sys.exit(1)
# are there at least two items?
if len(items) < 2:
    print 'Not enough items. Exiting program'
    sys.exit(1)

# the function 'fillIn' takes a sentence and replaces
# the word 'location' for an actual location
# and replaces the word 'item' for an actual item
# as defined in the files:
# locations.txt
# and items.txt
def fillIn(sentence):
    #shuffle the items and the locations
    random.shuffle(items)
    random.shuffle(locations)
    random.shuffle(names)
    #fill in the locations and items in the sentence
    # the counters are used so an item or location is not used twice
    # hence the shuffeling for randomization
    itemCounter         = 0
    locationCounter     = 0
    nameCounter         = 0
    finalSentence       = []
    for word in sentence.split(' '):
        # fill in a location
        if word == 'LOCATION':
            finalSentence.append( locations[locationCounter] )
            locationCounter += 1
        # or an item
        elif word == 'ITEM':
            finalSentence.append( items[itemCounter] )
            itemCounter += 1
        # is it a name?
        elif word == 'NAME':
            finalSentence.append( names[nameCounter] )
            nameCounter += 1
        # perhaps a location with a comma or dot?
        elif word[:-1] == 'LOCATION':
            finalSentence.append( locations[locationCounter] + word[-1])
            locationCounter += 1
        # or an item with a comma or dot or whatever
        elif word[:-1] == 'ITEM':
            finalSentence.append( items[itemCounter] + word[-1])
            itemCounter += 1
        # is it a namewith a comma, dot, whatever?
        elif word[:-1] == 'NAME':
            finalSentence.append( names[nameCounter] + word[-1] )
            nameCounter += 1
        # or else just the word
        else:
            finalSentence.append( word )
    # then make a sentence again out of the created list
    return ' '.join(finalSentence)


# the tests are defined here

def testOne():
    print '\n'
    print fillIn( random.choice(cat1Sentences) )
    print '\n\n'

# Category 2
def testTwo():
    print '\n'
    print fillIn( random.choice(cat2Sentences) )
    print '\n\n'

# Category 3
def testThree():
    print 'This is the situation for category 3, press enter for the question.\n\n'
    situation = random.choice( cat3Situations )
    print situation[0].split(':')[1]
    raw_input()
    print situation[1].split(':')[1]
    print '\n\n'


#############################################  MAIN LOOP ####################################


# ask the user which test this program should generate
def mainLoop():
    answer = 'begin'
    while True:
        answer = raw_input('Which category do you want to do?\nPossible answers are: 1, 2, 3 or q(uit)')
        if answer == 'q':
            print 'Exiting program.'
            sys.exit(1)
        elif answer == '1':
            print 'Category 1:\n',
            testOne()
        elif answer == '2':
            print 'Category 2:\n'
            testTwo()
        elif answer == '3':
            print 'Category 3:\n'
            testThree()
        else:
            print '\nNot a valid input, please try 1, 2, 3 or q(uit)\n'

if __name__ == "__main__":
    mainLoop()
