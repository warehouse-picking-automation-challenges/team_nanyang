#!/usr/bin/env python

import socket
import time
import rospy
import roslib
import sys
from std_msgs.msg import String
from demo.srv import *

HOST = "192.168.0.102"  #"172.22.135.106"#"192.168.0.102"      # The remote host
PORT = 30004                # The same port as used by the server

s = socket.socket()         # Create a socket object 
s.bind((HOST, PORT))        # Bind to the port 
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.  
time.sleep(0.5)	


msg=''

#print 'Connection address:', addr

'''
while 1:
    data = c.recv(1024)
    if not data: break
    print "received data:", data
    c.send(reply)  # echo
'''


def service_call(bin_num):
    rospy.wait_for_service('Call_Pick_n_Place')
    try:
        service_status = rospy.ServiceProxy('Call_Pick_n_Place', CallRobotToBin)
        feedback = service_status(bin_num)
        return feedback.str
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def bin_num_to_call(num):
	if (num != 0):
		print 'I will go to bin ' + str(num)
		msgg=service_call(str(num))
		#msgg='GO'		
		print msgg
		c.send(msgg)
		
	else: 
		print 'Invalid Command!!'
		msgg='SKIP'
		c.send(msgg)
	
def analyse_data(data):
	if (data == 'A'):
		bin_num_to_call(1)
	elif (data == 'B'):
		bin_num_to_call(2)
	elif (data == 'C'):
		bin_num_to_call(3)
	elif (data == 'D'):
		bin_num_to_call(4)
	elif (data == 'E'):
		bin_num_to_call(5)
	elif (data == 'F'):
		bin_num_to_call(6)
	elif (data == '#'):
		shutdown()
	else: bin_num_to_call(0)
			
def shutdown():
	data=''
	print 'I received a #'
	while (data !='SHUTDOWN'):	    	
		data = c.recv(8)
	print data
	
	c.close()
	print 'shutdown successfully' 
	

def receive_data():
	'''
	while 1:
		data=''
		reply_teckhou=''	    	
		data = c.recv(1)
		print data
	  	if not data: 
			#analyse_data(data)			
			break
		print "received data:", data
		reply_teckhou='ACK'
		c.send(reply_teckhou)  # echo
		analyse_data(data)
	'''
	data=''
	reply_teckhou=''
	while (data ==''):	    	
		data = c.recv(1)  	
	print "received data:", data
	reply_teckhou='ACK'
	c.send(reply_teckhou)  # echo
	analyse_data(data)	



if __name__ == '__main__': 
	rospy.init_node('Connection_Interface', anonymous=True)
	while not rospy.is_shutdown():	
		#print 'waiting'		
		receive_data()
		
	c.close()















