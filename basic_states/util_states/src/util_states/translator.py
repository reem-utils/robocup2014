#!/usr/bin/env python
# -.- coding: utf-8 -.-
# vim: expandtab ts=4 sw=4
# RECYCLE FROM:::
# A service that translates from location name (or object name) to location coordinates (and category)
# By Ricardo Tellez
# ricardo.tellez@pal-robotics.com
# 29-III-2012

import roslib
roslib.load_manifest('coord_translator')

#from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point32, Pose, Point, Quaternion

from coord_translator.srv import ObjectTranslator, LocationTranslatorResponse, ObjectTranslatorResponse, ObjectTranslatorDataBaseResponse, LocationTranslator, ObjectTranslatorDataBase
import rospy
from threading import Lock

PRINT_DETAILS = True
lock = Lock()  # To avoid race condition in the get_param when two nodes are accessing at the same time.


def translate_loc_to_coord(req):

    locationName = req.location
    lock.acquire(True)
    pois = rospy.get_param("/mmap/poi/submap_0")  # TODO: Make it global
    try:
        PRINT_DETAILS = rospy.get_param("coord_translator_print_info")
    except KeyError:
        PRINT_DETAILS = True

    lock.release()  # There aren't more get_param calls below, so we can release it here.

    if PRINT_DETAILS:
        print "Received location name: " + locationName
        rospy.loginfo("pois structure is like:    %s" % str(pois))
        rospy.loginfo("Type of data of pois: " + str(type(pois)))

    foundLocation = False
    answer = LocationTranslatorResponse()

    for key, value in pois.iteritems():
        not PRINT_DETAILS or rospy.loginfo(" Key: " + key + " Value: " + str(value))

        if value[1] == locationName:
            point = Point32(value[2], value[3], value[4])
            answer.coordinates = point
            answer.submap = value[0]
            foundLocation = True
            answer.exists = foundLocation
            if PRINT_DETAILS:
                rospy.loginfo("This list element is:\n " + str(value))
                print "LOCATION FOUND!"
            break

    '''
    for p in pois:
        poi = pois[p]
        rospy.loginfo("POIS %s"%str(poi))
        rospy.loginfo("LOCATION NAME ::%s"%locationName)
        rospy.loginfo("POIS $$$1$$$ %s"%str(poi[1]))
        if poi[1] == locationName:
            point = Point32 (poi[2],poi[3],poi[4])
            answer.coordinates = point
            answer.submap = poi[0]
            foundLocation = True
            print "LOCATION FOUND!"
            break
    '''
    answer.exists = foundLocation

    if PRINT_DETAILS:
        print answer
    return answer


def translate_obj_to_coord(req):
    objectName = req.objname

    lock.acquire(True)
    objects = rospy.get_param("/objects/objects_data")  # TODO: Make it global
    
    try:
        PRINT_DETAILS = rospy.get_param("coord_translator_print_info")
    except KeyError:
        PRINT_DETAILS = True

    if PRINT_DETAILS:
        print "Received object name: " + objectName

    foundObject = False
    answer = ObjectTranslatorResponse()

    for p in objects:
        oneObject = objects[p]
        if oneObject[0] == objectName:
            #Get the position of the room which's name is in oneObject[2]
            room_pois = rospy.get_param("/mmap/poi/submap_0/")  # TODO: Make it global
            for key, value in room_pois.iteritems():
                if value[1] == oneObject[2]:
                    answer.base_coordinates = Point(value[2], value[3], value[4])
                    break
            else:  # The loop finished without break
                rospy.logerr("\033[91mROOM %s NOT FOUND IN THE '/mmap/poi/submap_0/' OF THE PARAM SERVER!!\033[0m" % oneObject[2])
                answer.exists = False
                return answer

            arm_param = rospy.get_param("/objects/location_arm_poses/" + oneObject[3])  # TODO: Make it global
            armpose = Pose()
            armpose.position = Point(arm_param[0], arm_param[1], arm_param[2])
            armpose.orientation = Quaternion(arm_param[3], arm_param[4], arm_param[5], arm_param[6])
            answer.arm_coordinates = armpose
            answer.category = oneObject[1]
            answer.databaseID = oneObject[4]
            foundObject = True
            if PRINT_DETAILS:
                print "OBJECT FOUND!"
            break

    answer.exists = foundObject
    lock.release()
    if PRINT_DETAILS:
        print answer
    return answer


def translate_databaseID_to_obj(req):
    databaseID = req.databaseID
    lock.acquire(True)
    objects = rospy.get_param("/objects/objects_data")  # TODO: Make it global

    print "Received databaseID: " + str(databaseID)

    foundObject = False
    answer = ObjectTranslatorDataBaseResponse()

    for p in objects:
        oneObject = objects[p]
        if oneObject[4] == databaseID:
            answer.objname = oneObject[0]
            #Get the position of the room which's name is in oneObject[2]
            room_pois = rospy.get_param("/mmap/poi/submap_0/")  # TODO: Make it global
            for key, value in room_pois.iteritems():
                if value[1] == oneObject[2]:
                    answer.base_coordinates = Point(value[2], value[3], value[4])
                    break
            else:  # The loop finished without break
                rospy.logerr("\033[91mROOM %s NOT FOUND IN THE '/mmap/poi/submap_0/' OF THE PARAM SERVER!!\033[0m" % oneObject[2])
                answer.exists = False
                return answer

            arm_param = rospy.get_param("/objects/location_arm_poses/" + oneObject[3])  # TODO: Make it global
            armpose = Pose()
            armpose.position = Point(arm_param[0], arm_param[1], arm_param[2])
            armpose.orientation = Quaternion(arm_param[3], arm_param[4], arm_param[5], arm_param[6])
            answer.arm_coordinates = armpose
            answer.category = oneObject[1]
            foundObject = True
            print "OBJECT FOUND!"
            break

    answer.exists = foundObject
    lock.release()
    print answer
    return answer


def translator_server(): 
    rospy.init_node('translator_server')
    s = rospy.Service('loc_translator', LocationTranslator, translate_loc_to_coord)
    print "Ready to translate locations"
    s = rospy.Service('object_translator', ObjectTranslator, translate_obj_to_coord)
    print "Ready to translate Objects"
    s = rospy.Service('object_translator_dataBase', ObjectTranslatorDataBase, translate_databaseID_to_obj)
    print "Ready to translate dataBaseIDs"

    rospy.spin()

if __name__ == "__main__":
    # TODO: add a parameter to specify the poi param name
    translator_server()
