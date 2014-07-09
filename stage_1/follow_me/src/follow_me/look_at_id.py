#! /usr/bin/env python
'''
@author: Sam Pfeiffer
'''
# System stuff
import sys

# ROS stuff
import rospy

# Msgs
from pipol_tracker_pkg.msg import person, personArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Int32
from control_msgs.msg import PointHeadActionGoal

ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

PEOPLE_TRACKER_TOPIC = '/pipol_tracker_node/peopleSet'
ID_TO_FOLLOW_TOPIC = '/follow_me/id'
POINT_HEAD_TOPIC = '/head_controller/point_head_action/goal'

TIME_BETWEEN_GOALS = 0.3


class FollowPipolTrackerIdWithHead():
    """
    Subscribe to face detections
    and publish a personArray and also
    a PointStamped with the 3D pose of the person.
    """
    
    def __init__(self):
        rospy.loginfo("Setting publisher to '" + POINT_HEAD_TOPIC + "'")
        self.point_head_pub = rospy.Publisher(POINT_HEAD_TOPIC, PointHeadActionGoal)

        rospy.loginfo("Setting subscriber to '" + PEOPLE_TRACKER_TOPIC + "'")
        self.pipol_tracker_sub = rospy.Subscriber(PEOPLE_TRACKER_TOPIC, personArray, self.people_tracker_cb, queue_size=1)
        
        rospy.loginfo("Setting subscriber to '" + ID_TO_FOLLOW_TOPIC + "'")
        self.current_tracked_id = None
        self.id_to_follow_sub = rospy.Subscriber(ID_TO_FOLLOW_TOPIC, Int32, self.tracking_cb)
        
        self.last_pipol_msg = None
        self.new_msg = False
        rospy.loginfo("Waiting for first pipol tracker msg...")
        while self.last_pipol_msg == None:
            rospy.sleep(0.1)
        rospy.loginfo("Got one, node prepared!")
        self.curr_goal_to_send = None


    def people_tracker_cb(self, data):
        """cb for pipol tracker topic"""
        self.last_pipol_msg = data
        self.new_msg = True

    def tracking_cb(self, data):
        self.current_tracked_id = data.data
        rospy.loginfo("Following id: " + str(self.current_tracked_id) + " with head.")

    def run(self):
        """Publishing in topics (depending on rate of publication on detections)"""
        while not self.current_tracked_id:
            rospy.sleep(0.1)
            
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            id_is_in_msg = True
            max_lost_iterations = 30
            curr_lost_iterations = 0
            if id_is_in_msg:
                if self.new_msg:
                    self.new_msg = False
                    found_id_in_msg = False
                    for person in self.last_pipol_msg.peopleSet:
#                         print "person is: " + str(person)
#                         print "self.current_tracked_id is: " + str(self.current_tracked_id)
                        if int(person.targetId) == int(self.current_tracked_id):
                            found_id_in_msg = True
                            phg = PointHeadActionGoal()
                            phg.goal.min_duration = rospy.Duration(0.6) # adapt for as far as the detection is??
                            phg.goal.target.header.frame_id = "base_link"
                            phg.goal.target.header.stamp = rospy.Time.now()
                            phg.goal.target.point.x = person.x
                            phg.goal.target.point.y = person.y
                            phg.goal.target.point.z = 1.7 # always 1.7meters of Z
                            phg.goal.pointing_axis.x = 1.0
                            phg.goal.pointing_frame = 'head_mount_xtion_rgb_frame'
    
                            #Publish
                            if rospy.Time.now() - last_time > rospy.Duration(TIME_BETWEEN_GOALS):
                                self.point_head_pub.publish(phg)
                                last_time = rospy.Time.now()

                    if not found_id_in_msg: # If we lost the id stop
                        curr_lost_iterations += 1
                        if curr_lost_iterations > max_lost_iterations:
                            id_is_in_msg = False
                else:
                    rospy.sleep(0.1)
            else:
                rospy.logerr("We lost the id " + str(self.current_tracked_id) + "!")

    

if __name__ == '__main__':
#     if len(sys.argv) != 2:
#         print "Usage: " + sys.argv[0] + " <id_to_follow>"
#         print "Example:\n"
#         print sys.argv[0] + " 12"
#         exit(0)
    rospy.init_node('follow_vision_raw')
    rospy.sleep(0.5)  # Give a moment to init_node to do its job
    rospy.loginfo("Initializing follow id")
    follow_person = FollowPipolTrackerIdWithHead()
    rospy.loginfo("Running")
    follow_person.run()

