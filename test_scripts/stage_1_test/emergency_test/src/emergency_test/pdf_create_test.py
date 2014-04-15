#! /usr/bin/env python
# -*- coding: utf-8 -*-

import smach
import rospy
import rosparam
import smach_ros
from emergency_situation.GeneratePDF_State import GeneratePDF_State
from geometry_msgs.msg import PoseWithCovarianceStamped

class DummyStateMachine(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['succeeded', 'preempted', 'aborted'])

	def execute(self, userdata):
		rospy.sleep(1)
		return 'succeeded'

def main():
	sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

	with sm:
		smach.StateMachine.add(
			'Dummy',
			DummyStateMachine(),
			transitions={'succeeded':'Emergency_Create','aborted':'Emergency_Create', 'preempted':'Emergency_Create'})
		sm.userdata.person_location = PoseWithCovarianceStamped()
		sm.userdata.person_location.pose.pose.position.x = 1.0
		sm.userdata.person_location.pose.pose.position.y = 1.0
		smach.StateMachine.add(
			'Emergency_Create',
			GeneratePDF_State(),
			transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})

	sis = smach_ros.IntrospectionServer('create_pdf_introspection', sm, '/create_pdf_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	rospy.init_node('Emergency_PDF_Creator')
	main()