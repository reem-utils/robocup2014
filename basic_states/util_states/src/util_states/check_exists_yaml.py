#! /usr/bin/env python
import string
import os
import rospkg
import sys
"""
    Checks if objects in the yaml file are included in the roboList from the Robocup
"""
def check_yaml_compare(yamlFilePath, file_to_compare):
    f = open(yamlFilePath, 'r')
    f_comp = open(file_to_compare, 'r')
    
    end = False
    #Reading the file to compare with yaml
    name_field = []
    value_field = []
    
    while not end:
        line = f_comp.readline()
        name,value = line.partition(':')[::2]
        value = value[0:string.find(value, '\n')]
        valueArray = value.strip(' ').split(' | ')
        
        if len(value) == 0:
            end = True
        else:
            print "NAME : " + name
            print "VALUE : " + str(valueArray)
            name_field.append(name)
            for val in valueArray:
                value_field.append(val)
    print "VALUE ARRAY: " + str(value_field)
            
    #Comapare with yaml
    end = False
    in_yaml = True
    yaml_values = []
    while not end:
        line = f.readline()
        if '[' in line:
            name, value = line.partition(':')[::2]
            name = name.strip(' ')
            print "YAML VALUE: " + value
            print "YAML NAME: " + name
            yaml_values.append(name)
        if line=='':
            end = True
    
    for val in value_field:
        if val in yaml_values:
            print "OBJECT IN ROBOCUPLIST: " + val + " Exists in yaml"
        else:
            print "OBJECT IN ROBOCUPLIST: " + val + " DOES NOT Exists in yaml"
            in_yaml = False
    
    return in_yaml
        
        

def main():
    if(len(sys.argv) > 3):
        yamlName = sys.argv[1]
        file_to_compare_name = sys.argv[2]
        package_folder = sys.argv[3] 
    else:
        yamlName = 'pois_cocktail_party'
        file_to_compare_name = 'roboList'
        package_folder = "cocktail_party"
    
    
    rospack_instance = rospkg.RosPack()
    file_path = rospack_instance.get_path(package_folder)
    filePath = os.path.expanduser(file_path) + "/config/"
    check_yaml_compare(filePath + yamlName + ".yaml", filePath + file_to_compare_name)
    
if __name__ == '__main__':
    main()    