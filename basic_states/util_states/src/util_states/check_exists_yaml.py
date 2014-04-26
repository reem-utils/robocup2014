#! /usr/bin/env python
import string
import os
import rospkg
import sys

def check_yaml_compare(yamlFilePath, file_to_compare, package_folder):
    """
    Checks if objects from the Robocup file are included in the yaml file.
    
    Parameters:
    @param: yamlFilePath: the name of the .yaml File (without .yaml)
    @param: file_to_comapre: the name of the file to compare, usually roboList (should be included in the /config folder)
    @param: package_folder : the name of the package
    
    Returns:
    @return True: if the object exists
    @return: False: if the object Does Not exist
    """
    
    rospack_instance = rospkg.RosPack()
    file_path = rospack_instance.get_path(package_folder)
    filePath = os.path.expanduser(file_path) + "/config/"
    
    f = open(filePath + yamlFilePath + ".yaml", 'r')
    f_comp = open(filePath + file_to_compare, 'r')
    
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
        package_folder_name = sys.argv[3] 
        print "SYS"
    else:
        yamlName = 'pois_cocktail_party'
        file_to_compare_name = 'roboList'
        package_folder_name = "cocktail_party"
    
    
    check_yaml_compare(yamlName,file_to_compare_name, package_folder_name)
    
if __name__ == '__main__':
    main()    