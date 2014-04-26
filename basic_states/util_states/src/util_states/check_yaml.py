#! /usr/bin/env python
import string
import os
import rospkg
import sys

from util_states.colors import Colors

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
            name_field.append(name)
            for val in valueArray:
                value_field.append(val)
            
    #Comapare with yaml
    end = False
    in_yaml = True
    not_in_yaml = []
    while not end:
        line = f.readline()
        if '[' in line:
            name, value = line.partition(':')[::2]
            name = name.strip(' ')
            if not str(name) in value_field:
                in_yaml = False
                not_in_yaml.append(name)
                
        if line=='':
            end = True
    if in_yaml:
        print Colors().GREEN + "Dictionary OK!" + Colors().NATIVE_COLOR
    else:
        print Colors().RED + "The following objects do not exist in the file: " + str(not_in_yaml) + Colors().NATIVE_COLOR
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