#!/usr/bin/env python
# Echo client program
import socket
import time
import roslib
import tf
import rospy
from std_msgs.msg import String
#from demo.srv import *


HOST = "192.168.0.101"      # The remote host
PORT = 30002                # The same port as used by the server
print("Starting Program now")


s = socket.socket()         # Create a socket object 
s.bind((HOST, PORT))        # Bind to the port 
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.
	    
time.sleep(1)	


Q_bin = ['','','','','','','','','','','','','']
Q_retract = ['','','','','','','','','','','','','']
old_y =''
current_bin=''
count=0
previous_y=''
bin_teckhou='0'
'''
##########------UR_10-------##########################################################
#Q_initial = "( 0.49035, 0.000259784, 0.715819, 0.90128, 1.34798, 0.875023, 1.0 )"
Q_initial = "( 2.47, -2.15, 1.807, 0.317, 0.93, -0.408, 2.0 )"
Q_order_1 = '( 0.7251, -1.62542, 1.71749, 0.20016, 1.18003, -0.415467, 2.0)'
Q_order_2 = '( -0.30564, -0.536796, 0.0945618, 1.86758, -0.129146, -0.515766, 1.0 )'
#######################################################################################
'''
##########------UR_5-------##########################################################
Q_initial = "( -0.556398, -2.07768, 2.30013, -3.36982, -1.00025, -0.748806, 2.0 )"
Q_order_1 = '( -2.03936, -0.678519, 1.27867, -2.1364, -1.60137, -0.74871, 2.0 )'
Q_order_2 = '( 0.241629, 0.715323, -0.269696, 3.068, 0.431413, 0.00517245, 1.0  )'
#######################################################################################

##############################definition of functions###############################


def service_GoToBin(req):
	print req.str
		
	if (req.str != '0'):
		pick_n_place(int(req.str))

	return 'GO'

def service_server():
    	s = rospy.Service('Call_Pick_n_Place', CallRobotToBin, 	service_GoToBin)
    	
    	rospy.spin()



def change_pose_value(pose,direction,value):
	if (direction == 'x'):
		pose[0]=pose[0] + (value)
	elif (direction == 'y'):
		pose[1]=pose[1] + (value)
	elif (direction == 'z'):
		pose[2]=pose[2] + (value)
	elif (direction == 'rx'):
		pose[3]=pose[3] + (value)
	elif (direction == 'ry'):
		pose[4]=pose[4] + (value)
	elif (direction == 'rz'):
		pose[5]=pose[5] + (value)
	text=''	
	text = Convert_To_UR_Format(pose)
	return text
	

def Convert_To_UR_Format(pose):
	text=''	
	text= '('+ str(pose[0])+ ','+str(pose[1])+ ','+ str(pose[2])+ ','+ str(pose[3])+ ','+ str(pose[4])+ ','+ str(pose[5])+ ','+ str(pose[6])+ ')'
	return text

def state_check():
	msg = ""
	current_pose=""	
	while (msg != "OK"):
      		msg = c.recv(1024)
	current_pose = c.recv(1024)
	#print "Current position :  " + current_pose

def pick_n_place(bin_number):
	c.send(Q_initial);
	#print("Initial position sent.")
	state_check()

	#c.send(b"(-0.464151, -0.890661, 0.915942, 1.49941, -0.30745, 2.04508)");
	c.send(Q_retract[bin_number]);
	#print("Waypoint_1 sent.")
	state_check()
	
	#c.send(b"(0.789362, -0.62252, 0.914084, -1.36727, -0.725173, -2.09093)");
	c.send(Q_bin[bin_number]);
	#print("Waypoint_2 sent.")			
	state_check()	
	
	gripper(True)
	time.sleep(4.0)
	

	c.send(Q_retract[bin_number]);
	#print("Waypoint_3 sent.")
	state_check()	

	if (bin_number!= 3 and bin_number!= 6):
		c.send(Q_initial);
		#print("Initial position sent.")
		state_check()


	c.send(Q_order_1);
	#print("Order_1 position sent.")
	state_check()

	c.send(Q_order_2);
	#print("Order_2 position sent.")
	state_check()
	
	gripper(False)
	#time.sleep(0.5)
	time.sleep(3)
	
	c.send(Q_order_1);
	#print("Order_1 position sent.")
	state_check()

	c.send(Q_initial);
	#print("Initial position sent.")
	state_check()

def sent_current_pose():
	pub = rospy.Publisher('UR_current_pose', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
	for i in range(1,13):
		my_msg= Q_bin[i]
       		#rospy.loginfo(hello_str)
        	#pub.publish(hello_str)
 		#rospy.loginfo(my_msg)
        	pub.publish(my_msg)
		rospy.loginfo("published!!")
		
        	#rate.sleep()

def confirm_to_move():
	global count
	global old_y	
	global previous_y
	####################################################
	print 'count: ' + str(count)	
	print 'old: ' + old_y
	print 'current: ' + current_bin
	
	if (old_y == current_bin):
		count =count +1
		#old_y = current_bin
	else:	
		count = 0
		old_y=current_bin
	
	if (count == 10):

		count = 0
		if (previous_y != current_bin):
			if (current_bin == '1'):
				print '1'
				pick_n_place(1)
				old_y = '1'
				previous_y='1'
				time.sleep(1.0)
				
			elif (current_bin == '2'):
				print '2'
				pick_n_place(2)
				old_y = '2'
				previous_y='2'
				time.sleep(1.0)
			elif (current_bin == '3'):
				print '3'
				pick_n_place(3)
				old_y = '3'
				previous_y='3'
				time.sleep(1.0)
	

	####################################################

def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	#print data.data
	global current_bin
	current_bin=str(data.data)
	#confirm_to_move()
		
	'''
	global old_y
	####################################################
	current_bin=str(data.data)
	
	#print 'old: ' + old_y
	print 'current: ' + current_bin
	if (old_y != current_bin):
			if (current_bin == '1'):
				print '1'
				pick_n_place(1)
				old_y = '1'
				time.sleep(3.0)
				
			elif (current_bin == '2'):
				print '2'
				pick_n_place(2)
				old_y = '2'
				time.sleep(3.0)
			elif (current_bin == '3'):
				print '3'
				pick_n_place(3)
				old_y = '3'
				time.sleep(3.0)
	

	####################################################
	'''
    
def get_target_pose():
	rospy.init_node('kj_pick_n_place', anonymous=True)
    	rospy.Subscriber("pick_up_bin", String, callback)
	
	
    # spin() simply keeps python from exiting until this node is stopped
    	#rospy.spin() 

def gripper(state):
	if (state == True):
		c.send(b"(0, 0, 0, 0, 0, 0, 3)")
	elif (state == False):
		c.send(b"(0, 0, 0, 0, 0, 0, 4)")

####################################################################



def kj_pick_n_place():
	
	
	'''
	for i in range (1,13):
		Q_bin[i]=Convert_To_UR_Format(rospy.get_param('send_point/point/Q' + str(i)))
		Q_retract[i]=change_pose_value((rospy.get_param('send_point/point/Q' + str(i))),'x',(-0.2))
	'''
	


	####################-----START----#######################################
	
	#for i in range(1,13):
	#	pick_n_place(i)
	
	for i in range(1,7):
		pick_n_place(i)

	#pick_n_place(1)

	


	####################----- END----#######################################	
	#c.close()
	

	print("Program finish")

'''
def listern_to_ar():
	
	
	listener = tf.TransformListener()
	old_y=0
	bin_y=0
    	rate = rospy.Rate(1.0)
	
    	while not rospy.is_shutdown():
        	try:	
			
            		#(trans,rot) = listener.lookupTransform('/kinect_rgb_optical_frame', '/ar_tag', rospy.Time(0))
			(trans,rot) = listener.lookupTransform('/world_frame', '/ar_tag', rospy.Time(0))
			
        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            		continue 

		#print 'X: '+ str(trans[0]) + '  Y: '+ str(trans[1]) + '  Z: '+ str(trans[2]) 
	
		
		if (trans[1]<=0.4 and trans[1]>=0.06 ):
			
			#print '1'
			bin_y=1
		elif (trans[1]<=0.06 and trans[1]>=-0.24 ):
				
			#print '2'
			bin_y=2
		elif (trans[1]<=-0.24 and trans[1]>=-0.4 ):
				
			#print '3'
			bin_y=3
		
		if (old_y != bin_y):
			if (bin_y == 1):
				print '1'
				pick_n_place(1)
				
				old_y=1
			elif (bin_y == 2):
				print '2'
				pick_n_place(2)
				
				old_y=2
			elif (bin_y == 3):
				print '3'
				pick_n_place(3)
				
				old_y=3



		

        	rate.sleep()
'''
def testing_gripper():
	
	rospy.init_node('kj_pick_n_place', anonymous=True)	
	while not rospy.is_shutdown():	
		gripper(True)
		time.sleep(2.0)
		gripper(False)	
		time.sleep(2.0)

def get_command_from_teckhou():	
	sub_teckhou()
	if (bin_teckhou != '0'):
		pick_n_place(int(bin_teckhou))

def callback_teckhou(data):
	global bin_teckhou
	bin_teckhou=data.data
	

def sub_teckhou():
	rospy.Subscriber("move_robot", String, callback_teckhou)
	

if __name__ == '__main__': 
	rospy.init_node('kj_pick_n_place', anonymous=True)	
	for i in range (1,7):
		Q_bin[i]=Convert_To_UR_Format(rospy.get_param('send_point/point/Q' + str(i)))
		Q_retract[i]=change_pose_value((rospy.get_param('send_point/point/Q' + str(i))),'x',(0.32))
	while not rospy.is_shutdown():		
		kj_pick_n_place()
		#get_command_from_teckhou()
		#service_server()   ############later open this line only########################
	'''
		get_target_pose()
		confirm_to_move()
	#kj_pick_n_place()
		#sent_current_pose()
	#listern_to_ar()
	'''
	
		





