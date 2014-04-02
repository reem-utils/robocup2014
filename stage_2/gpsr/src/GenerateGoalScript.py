
import roslib

def printNewGoal(oaction='go_to', oitem='w', operson='e', olocation='kitchen',
    templatefile='_goal.soar', goalfile='goal-test'):
    templatesfilepath = roslib.packages.get_pkg_dir("gpsr") + "/src/goalSoars/"
    targetfilepath = roslib.packages.get_pkg_dir("gpsrSoar") + "/SOAR/gp/elaborations/"
    template = templatesfilepath + oaction + templatefile
    target = targetfilepath + goalfile + '.soar'
    tempfile = open(template, 'r')
    targfile = open(target, 'w')

    for line in tempfile.readlines():
        line = line.replace('ITEM', oitem)
        line = line.replace('PERSON', operson)
        line = line.replace('LOCATION', olocation)
        targfile.write(line)

    tempfile.close()
    targfile.close()
