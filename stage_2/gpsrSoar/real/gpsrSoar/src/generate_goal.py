

def printNewGoal(oaction='go_to', oitem='w', operson='e', olocation='kitchen', order='1',
    templatefile='goalTemplate.soar', goalfile='goal',
    filepath='/home/jy/reem_at_iri/trunk/state_machines/gpsr_applyOrder/SOAR_agents/REEM01/'):
    template = filepath + templatefile
    target = filepath + goalfile + order + '.soar'
    tempfile = open(template, 'r')
    targfile = open(target, 'w')

    for line in tempfile.readlines():
        line = line.replace('INDEX', order)
        line = line.replace('ITEM', oitem)
        line = line.replace('PERSON', operson)
        line = line.replace('LOCATION', olocation)
        line = line.replace('ACTION', oaction)
        targfile.write(line)

    tempfile.close()
    targfile.close()
