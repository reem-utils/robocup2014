#! /usr/bin/env python
'''
@author: Luca Marchionni
'''
# System stuff
import sys
import math

# ROS stuff
import rospy

# Msgs
from pipol_tracker_pkg.msg import person, personArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import LaserScan

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

PEOPLE_TRACKER_TOPIC = '/pipol_tracker_node/peopleSet'
ID_TO_FOLLOW_TOPIC = '/follow_me/id'
INPUT_LASER_TOPIC = '/scan_filtered'
OUTPUT_LASER_TOPIC = '/pipol_filtered_scan'


class PipolLaserFilter():
    """
    Subscribe to laser, personArray and id
    """
    
    def __init__(self):
        self.laser_x = 0.08
        self.last_pipol_msg = None
        self.last_scan_msg = None
        self.new_msg = False
        self.current_tracked_id = None

        rospy.loginfo("Setting publisher to '" + OUTPUT_LASER_TOPIC + "'")
        self.filtered_laser_pub = rospy.Publisher(OUTPUT_LASER_TOPIC, LaserScan)

        rospy.loginfo("Setting subscriber to '" + INPUT_LASER_TOPIC + "'")
        self.laser_sub = rospy.Subscriber(INPUT_LASER_TOPIC, LaserScan, self.laser_cb, queue_size=1)

        rospy.loginfo("Setting subscriber to '" + PEOPLE_TRACKER_TOPIC + "'")
        self.pipol_tracker_sub = rospy.Subscriber(PEOPLE_TRACKER_TOPIC, personArray, self.people_tracker_cb, queue_size=1)
        
        rospy.loginfo("Setting subscriber to '" + ID_TO_FOLLOW_TOPIC + "'")
        self.id_to_follow_sub = rospy.Subscriber(ID_TO_FOLLOW_TOPIC, Int32, self.tracking_cb)
        

        rospy.loginfo("Waiting for first pipol tracker msg...")
        while self.last_pipol_msg == None:
            rospy.sleep(0.1)
        rospy.loginfo("Got one, node prepared!")

    def laser_cb(self, data):
        """cb for laser topic"""
        print "laser cb"
        self.last_scan_msg = data
        self.new_msg = True

    def people_tracker_cb(self, data):
        print "people tracker cb"
        """cb for pipol tracker topic"""
        self.last_pipol_msg = data
        
    def tracking_cb(self, data):
        print "people id cb"
        self.current_tracked_id = data.data
        rospy.loginfo("Following id: " + str(self.current_tracked_id) + " with head.")

    def run(self):
        """Publishing in topics (depending on rate of publication on detections)"""
        while not self.current_tracked_id:
            rospy.sleep(0.1)
            
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():            
            if self.new_msg:
                self.new_msg = False
                current_scan_to_send = self.last_scan_msg
                if self.current_tracked_id is not None:
                    for person in self.last_pipol_msg.peopleSet:
                        if int(person.targetId) == int(self.current_tracked_id):
                            print "ok"
                            person_angle = math.atan2(person.y, person.x)
                            
                            nr = []
                            for (i, ray) in enumerate(self.last_scan_msg.ranges):
                                angle = self.last_scan_msg.angle_min + self.last_scan_msg.angle_increment*i
                                angle = -angle #because laser is flipped 180 deg
                                dx = math.cos(angle) * ray + self.laser_x
                                dy = math.sin(angle) * ray
                                
                                if math.sqrt((person.x - dx)**2 + (person.y-dy)**2) < 0.5:
                                    nr.append(0.001) #small distance to filter it out from scan
                                else:
                                    nr.append(ray)
                 
                            current_scan_to_send.ranges = nr

                self.filtered_laser_pub.publish(current_scan_to_send)
    

if __name__ == '__main__':
#     if len(sys.argv) != 2:
#         print "Usage: " + sys.argv[0] + " <id_to_follow>"
#         print "Example:\n"
#         print sys.argv[0] + " 12"
#         exit(0)
    rospy.init_node('follow_vision_raw')
    rospy.sleep(0.5)  # Give a moment to init_node to do its job
    rospy.loginfo("Initializing follow id")
    filter_person_in_laser = PipolLaserFilter()
    rospy.loginfo("Running")
    filter_person_in_laser.run()

