#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import roslib
from std_msgs.msg import String

from item_library_new import *
from mantis.srv import *

def UR5_Service_Call():
	t = [0] * 10
	rospy.wait_for_service('gripper_comm')
	
	try:
		service_status = rospy.ServiceProxy('gripper_comm', SRV_Gripper)
		a = input("Command: " )
		b = input("Item ID: " ) 		
		
		item, name = target_dimension( b )
		print name
		
		t[0] = item[0]
		t[1] = item[1]
		t[2] = item[2]

		t[3] = 0.23
		t[4] = 0.11
		t[5] = 2.36
		
		t[6] = item[0]
		t[7] = item[1]
		t[8] = item[2]
		
		feedback = service_status(a, b, t)
		
		print "Status: " + str(feedback.status)
		print "Pose before: " + str(t)
		print "Pose after: " 

		i = 0
		while i < 40:
			print feedback.grasping_pose[i], feedback.grasping_pose[i+1], feedback.grasping_pose[i+2], feedback.grasping_pose[i+3], feedback.grasping_pose[i+4], feedback.grasping_pose[i+5], feedback.grasping_pose[i+6], feedback.grasping_pose[i+7] 
			i += 8
		print "\n"

		print "Method: " + str(feedback.method)
	
	except rospy.ServiceException, e:
		print "Error"

if __name__ == '__main__':
	UR5_Service_Call()

