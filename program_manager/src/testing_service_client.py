#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import roslib
from std_msgs.msg import String
from object_recognition.srv import *


def UR5_Service_Call(msg):
   
    rospy.wait_for_service('identify')
    try:
        service_status = rospy.ServiceProxy('identify', SRV_Identification)
        feedback = service_status(msg)
        print "name_index: " + feedback.name_index  
        print "confidence: " + feedback.confidence
        print "blob_index: " + feedback.blob_index
        
        return feedback.confidence
    except rospy.ServiceException, e:
        print "UR5 Service call failed: %s"%e




if __name__ == '__main__':

	
	print "First set of data:"
	a=UR5_Service_Call('test_data_conghui')
	print "Second set of data:"
	a=UR5_Service_Call('test_data_kj')
        
      
    
