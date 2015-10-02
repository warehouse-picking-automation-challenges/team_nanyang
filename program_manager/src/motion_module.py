#!/usr/bin/env python
# Echo client program
import socket
import time
import roslib
import tf
import math
import rospy
from std_msgs.msg import String
from program_manager.srv import *


############################################################################################################
#########################----------Setting----------########################################################
############################################################################################################
move_out_of_bin_mode=1      ### 1=end effector axis, 2= global x-axis
gripper_type=1              ### 0= no gripper; 1=mantis gripper



##########------UR_5 Preset Joint Value-------#################
if (gripper_type==0):
    Q_initial_normal_bin = "()"
    Q_order_1 = '()'
   
elif (gripper_type==1):
    gripper_weight_no_payload = 2.8
    gripper_weight_with_payload = 4.2
    gripper_COG_Z_no_payload = 0.123
    gripper_COG_Y_no_payload = -0.088
    gripper_COG_Z_with_payload = 0.173
    gripper_COG_Y_with_payload = -0.065
    Q_initial_normal_bin = "( -0.541058, -1.78772, 2.34035, -0.500496, 1.01565, -3.15614, 2.0 ,0.2 ,0,0,0,0,0,0)"
    Q_initial_bin_7_10 = "(-1.7384, -2.45732, -2.26618, -1.57594, -0.179498, 3.17085,2.0 ,0.2 ,0,0,0,0,0,0)"
    Q_order_down='( -2.07546, -1.78772, 2.34035, -0.500496, 1.01565, -3.15614, 2.0, 0.2 ,0,0,0,0,0,0)'
    Q_order_1 = '(-1.65863, -0.458556, 1.35191, -0.873849, 1.53043, -3.15435, 2.0, 0.2 ,0,0,0,0,0,0)'
    Q_order_bin_7_10 = '(1.74499, -2.46703, -1.85691, -1.98349, -1.37349, 3.17048, 2.0, 0.2 ,0,0,0,0,0,0)'
 
current_end_effector_position = [0,0,0,0,0,0]
init_position_EE_y=[0,0.1065,0.1065,0.1065,0.088,0.088,0.088,0.088,0.088,0.088,0.1065,0.1065,0.1065]
init_position_EE_z=[0,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08,-0.08]

#init_offset_z=[0,-0.05,-0.05,-0.05,-0.05,-0.05,-0.05,-0.07,-0.05,-0.05,-0.07,-0.05,-0.05]######ori
init_offset_z=[0,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07,-0.07]
#init_offset_y=[0,0,0,0,0,0,0,0,0,0,0,0,0]##########ori
init_offset_y=[0,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02,-0.02]
kinect_capturing_distance = -0.09##########################################################################cccccccccc
EE_move_up_distance = -0.02   # move up EE after grasping in the bin before move out
EE_side_suction_move_up = 0.035 # move up EE after grasping in the bin before move out
EE_move_forward_distance_front_suction = 0.025

#init_position_EE_y=[0,0.1075,0.1075,0.1075]
#init_offset_z=[0,-0.05,-0.05,-0.05]###############
#init_offset_z=[0,0,0,0]###############
#init_offset_y=[0,0,0,0,0,0,0,0,0,0]
#init_offset_y=[0,-0.1075,-0.1075,-0.1075]###############

############################################################################################################


HOST = "192.168.0.104"      # The remote host
PORT = 30008             # The same port as used by the server
print("Program starting ...")


s = socket.socket()         # Create a socket object 
s.bind((HOST, PORT))        # Bind to the port 
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.
	    
time.sleep(1)	
print("Connected with UR 5 !")

Q_bin_new = ['','','','','','','','','','','','','']
Q_retract = ['','','','','','','','','','','','','']
current_pose=""	
suction_method=100
safety_per=False
count_fine_movement=0
step_fine_movement=0.0
axis_fine_movement=''




#############################################################################################################
#########################################--- Service----#####################################################
def service_GoToBin(req):
	global current_pose
	global suction_method
	suction_method =0
	status=False
	suction_method =req.suction_method
	print req.bin + " & action: " + req.action + " target_pose: " + str(req.target_pose) + " Suction method: " + str(req.suction_method)
	
	status=pick_n_place(int(req.bin),int(req.action),req.target_pose)
	print "I have executed pick n place."
	return SRV_MotionResponse(status,current_pose)

def service_server():
    s = rospy.Service('Call_UR5', SRV_Motion, 	service_GoToBin)
    print "UR5 service Waiting ..."
    rospy.spin()

##################################################################################
##################################################################################


############################################################################################################
#########################################--- Type of Action----################################################

def pick_n_place(bin_number,action,target_pose):
    global suction_method
    global count_fine_movement
    global step_fine_movement
    global axis_fine_movement
    global EE_move_up_distance
    global EE_side_suction_move_up

    if (action == 1):   # go to target bin ready for capture
        go_to_capturing_position(bin_number,False)
        return True

    elif (action == 2): # move robot to the item
	    move_to_item(bin_number,target_pose)
	    return True
		
    elif (action == 3): # go to init pose
        if bin_number == 7 or bin_number ==10:
            c.send(Q_initial_bin_7_10)
        else:
            c.send(Q_initial_normal_bin)
        state_check()
        return True
        
    elif (action == 4): # dummy, for testing purpose
        return True
    
    elif (action == 5): # move to grasping pose
        global safety_per
        Move_to_grasping_pose(bin_number,target_pose)
        return safety_per
        
    elif (action == 6): # turn on suction
        time.sleep(0.5)
        suction(True)
        time.sleep(1)
        return True
        
    elif (action == 7): #turn off suction
        time.sleep(0.5)
        suction(False)
        time.sleep(1)
        return True
        
    elif (action == 8): #move to order bin 1st stage
        
        set_payload(True)
        
        if suction_method ==1 or suction_method ==2 or suction_method ==5 or suction_method ==8:
            total_length=count_fine_movement * (-1*step_fine_movement)
            move_end_effector(axis_fine_movement, total_length, 0.05)
            if suction_method==8:
                move_end_effector('y', (-0.03), 0.05) ## go up 3cm
                move_end_effector('rx', 4, 0.05) ## rotate up 4 degree
        if suction_method==1 or suction_method==3 or suction_method==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
            move_end_effector('rx', 2, 0.05)
            
        move_end_effector('ry', -1*math.degrees(target_pose[4]), 0.05)
        
        if  suction_method==1 or suction_method ==3 or suction_method ==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
            move_end_effector('y', EE_move_up_distance, 0.05) ## move up item 3cm
        
        if suction_method==5:
            if target_pose[5] > 0:
                move_end_effector('x', -1*EE_side_suction_move_up, 0.05) ## move out item   
            elif target_pose[5] < 0:
                move_end_effector('x', EE_side_suction_move_up, 0.05) ## move out item   
        move_end_effector('z', (-1*target_pose[2]+(-1*init_position_EE_z[bin_number])+(-1*kinect_capturing_distance)), 0.05) ## move out item
        

        return True

    elif (action == 9): # move to init pose from order bin
        #change_pose_value_and_move(convert_string_to_pose(current_pose),'z',0.3,0.25)
        set_payload(False)
        
        move_end_effector('y', -0.2, 0.6)
        c.send(Q_initial_normal_bin)
        state_check()
        return True
    
    elif (action == 10):    #move robot out of the bin, meant for error recovery
        set_payload(False)
        if suction_method ==1 or suction_method ==2 or suction_method ==5 or suction_method ==8:
            total_length=count_fine_movement * (-1*step_fine_movement)
            move_end_effector(axis_fine_movement, total_length, 0.05)
            if suction_method==8:
                move_end_effector('y', (-0.03), 0.05) ## go up 3cm
                move_end_effector('rx', 4, 0.05) ## rotate up 4 degree
        if suction_method==1 or suction_method==3 or suction_method==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
            move_end_effector('rx', 2, 0.05)
            
        move_end_effector('ry', -1*math.degrees(target_pose[4]), 0.05)
        
        if suction_method==1 or suction_method ==3 or suction_method ==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
            move_end_effector('y', EE_move_up_distance, 0.05) ## move up item 3cm
        
        if suction_method==5:
            if target_pose[5] > 0:
                move_end_effector('x', -1*EE_side_suction_move_up, 0.05) ## move out item
            elif target_pose[5] < 0:
                move_end_effector('x', EE_side_suction_move_up, 0.05) ## move out item
            
        #move_end_effector('z', -1*target_pose[2], 0.05) ## move out item
        if bin_number == 11 and suction_method != 5:
	            move_end_effector('z', (-0.06), 0.05) ## go to item
	            move_end_effector('x', -1*target_pose[0], 0.05)
	            move_end_effector('z', -1*target_pose[2]+0.06, 0.05) ## go to item
        else:
            move_end_effector('z', -1*target_pose[2], 0.05) ## move out item
        
        if suction_method==5:
            if (bin_number == 1 or bin_number == 4 or bin_number == 11 ) and target_pose[5]>0:
                move_end_effector('y', -0.22, 0.2)
                

        go_to_capturing_position(bin_number,True)
        
        return True

    
    elif (action == 11):    # checking the suction on/off
        state=check_suction_status()
        print "Suction status: " + str(state)
        return state
        
    elif (action == 12): #move to order bin 2nd stage
        if bin_number == 11 and suction_method != 5:
	            move_end_effector('z', (-0.06), 0.05) ## go to item
	            move_end_effector('x', -1*target_pose[0], 0.05)
	            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance)+0.06, 0.05) ## go to item
        else:
            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance), 0.05) ## move out item
        
        if suction_method==5:
            if (bin_number == 1 or bin_number == 4 or bin_number == 11) and target_pose[5]>0:
                move_end_effector('y', -0.22, 0.2)

        if bin_number == 1 or bin_number == 2 or bin_number == 4 or bin_number == 5 or bin_number == 8 :
            c.send(Q_initial_normal_bin)
            state_check()
            #move_end_effector_2_axis_translation('y', 0.10, 0.2, 'x', 0.1)
        if bin_number != 7 and bin_number != 10:
            c.send(Q_order_down)
            state_check()
        return True
        
    elif (action == 13): #move to order bin 3rd stage
        if bin_number ==7 or bin_number ==10:
            c.send(Q_initial_bin_7_10)
            state_check()
            c.send(Q_order_bin_7_10)
        else:
            c.send(Q_order_1)
        state_check()
        order_bin(bin_number)
        move_end_effector('y', 0.2, 0.6)
        return True

    elif (action == 14): #move to bin meant for error recovery
        set_payload(False)
        if bin_number == 11 and suction_method != 5:
            move_end_effector('z', (-0.06), 0.05) ## go to item
            move_end_effector('x', -1*target_pose[0], 0.05)
            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance)+0.06, 0.05) ## go to item
        else:
            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance), 0.05) ## move out item
        if suction_method==5:
            if (bin_number == 1 or bin_number == 4 or bin_number == 11 ) and target_pose[5]>0:
                move_end_effector('y', -0.22, 0.2)
        go_to_capturing_position(bin_number,True)
        return True
        
    elif (action == 15): #checking collision
        motion_approval=False
        motion_approval=collision_check(bin_number,target_pose,suction_method)
        return motion_approval

    elif (action == 16): #move to init meant for error recovery
        set_payload(False)
        if bin_number == 11 and suction_method != 5:
            move_end_effector('z', (-0.06), 0.05) ## go to item
            move_end_effector('x', -1*target_pose[0], 0.05)
            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance)+0.06, 0.05) ## go to item
        else:
            move_end_effector('z', (init_position_EE_z[bin_number])+(kinect_capturing_distance), 0.05) ## move out item
        if suction_method==5:
            if (bin_number == 1 or bin_number == 4 or bin_number == 11 ) and target_pose[5]>0:
                move_end_effector('y', -0.22, 0.2)
        if bin_number == 7 or bin_number ==10:
            c.send(Q_initial_bin_7_10)
        else:
            c.send(Q_initial_normal_bin)
        state_check()
        return True    
        


##################################################################################
##################################################################################


        


####################################################################################################
#################################--- Conversion----#################################################

def change_init_pose (pose,bin_num):
    global init_offset_y
    global init_offset_z
    
    
    convert_pose=[0,0,0,0,0,0]
    convert_pose_2=[0,0,0,0,0,0]
    pose_1=[0,0,0,0,0,0,0,0]
    pose_2=[0,0,0,0,0,0,0,0]
    new_pose=[0,0,0,0,0,0,0,0]
    for i in range (6):
        convert_pose[i]=pose[i]
    pose_1=get_end_effector_transform_pose(convert_pose,'z',init_offset_z[bin_num])
    
    for i in range (6):
        convert_pose_2[i]=pose_1[i]
        
    pose_2=get_end_effector_transform_pose(convert_pose_2,'y',init_offset_y[bin_num])
    
    for i in range (6):
        new_pose[i]=pose_2[i]
    new_pose[6]=pose[6]
    new_pose[7]=pose[7]
    text=""
    text=Convert_To_UR_Format(new_pose)
    
    
	
    return text
    
def get_end_effector_transform_pose(pose,axis,value):
    
    x_value='0'
    y_value='0'
    z_value='0'
    
    if axis =='x':
        x_value=str(value)
    elif axis=='y':
        y_value=str(value)
    elif axis=='z':
        z_value=str(value)
    send_data= "(" + x_value + "," + y_value  + "," + z_value + ",0,0,0 , 8, 0," + str(pose[0]) + "," + str(pose[1]) + "," + str(pose[2]) + "," + str(pose[3]) + "," + str(pose[4]) + "," + str(pose[5]) + ")"
    #print "send_data: "+send_data
    c.send(send_data)
    msg=""
    while (msg != "OK"):
      		msg = c.recv(2)
    get_pose=c.recv(1024)
    #print "get_pose: "+get_pose
    pose_test=convert_string_to_pose(get_pose)
    return pose_test   


def convert_string_to_pose(msg):
	pose=[ 0, 0, 0, 0, 0, 0,1,0]
	pose_test=[ '','' ,'' ,'' ,'' ,'' ]
	test,test_pose=msg.split('[')
	msg,test_pose2=test_pose.split(']')
	pose_test[0],pose_test[1],pose_test[2],pose_test[3],pose_test[4],pose_test[5]=msg.split(',')
	for i in range (6):
		pose[i]=float(pose_test[i])
	return pose
	

def change_pose_value(pose,direction,value,velocity):
	
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
	pose[7]=velocity
	text=''	
	text = Convert_To_UR_Format(pose)
	return text
	

def change_pose_value_and_move(pose,direction,value,velocity):
    text=""
    text=change_pose_value(pose,direction,value,velocity)
    c.send(text)
    state_check()


def Convert_To_UR_Format(pose):
	text=''	
	text= '('+ str(pose[0])+ ','+str(pose[1])+ ','+ str(pose[2])+ ','+ str(pose[3])+ ','+ str(pose[4])+ ','+ str(pose[5])+ ','+ str(pose[6])+  ','+ str(pose[7]) + ',0,0,0,0,0,0)'
	return text
	

def state_check():
	msg = ""
	global current_pose	
	while (msg != "OK"):
      		msg = c.recv(2)
	current_pose = c.recv(1024)


def convert_string_to_pose_eight_value(msg):
	pose=[ 0, 0, 0, 0, 0, 0,1,0]
	pose_test=[ '','' ,'' ,'' ,'' ,'','','' ]
	test,test_pose=msg.split('(')
	msg,test_pose2=test_pose.split(')')
	pose_test[0],pose_test[1],pose_test[2],pose_test[3],pose_test[4],pose_test[5],pose_test[6],pose_test[7]=msg.split(',')
	for i in range (8):
		pose[i]=float(pose_test[i])
	return pose    
    

##################################################################################
##################################################################################        
        
       

#######################################################################################################
#################################--- Control of UR----#################################################
def go_to_capturing_position(bin_num,error_recovery):
    global current_end_effector_position
    global kinect_capturing_distance
    
    current_end_effector_position = [0,0,0,0,0,0]
    
    set_payload(False)
    if error_recovery == False:
        if bin_num == 7 or bin_num ==10:
            c.send(Q_initial_bin_7_10)
        else:
            c.send(Q_initial_normal_bin)
        state_check()
    c.send(Q_bin_new[bin_num])
    state_check()
    
    move_end_effector('z', kinect_capturing_distance, 0.05)
    current_end_effector_position=[0,(init_position_EE_y[bin_num]),(init_position_EE_z[bin_num])+kinect_capturing_distance,0,0,0]
    print "current position: " + str(current_end_effector_position)
        

def Ave_force():
    total_force=0
    ave_force_feedback=0
    for i in range (10):
        time.sleep(0.005)
        aaa=read_tcp_force()
        total_force = total_force + aaa
    ave_force_feedback = total_force/10
    print "ave load: " + str(ave_force_feedback)
    return ave_force_feedback
        

def read_tcp_force():
    c.send(b"(0, 0, 0, 0, 0, 0, 11, 0,0,0,0,0,0,0)")
    msg = ""
    while (msg == ""  ):
        msg = c.recv(1024)
    print msg
    force=decode_tcp_force(msg)
    print "force: " + force
    return float(force)
        
def decode_tcp_force(data):
    a,b=data.split('[')
    a,c=b.split(']')
    b,c,d,e,f,g=a.split(',')
    return d

def set_payload(with_weight):
    global gripper_weight_with_payload
    global gripper_weight_no_payload
    global gripper_COG_Z_with_payload
    global gripper_COG_Y_with_payload
    global gripper_COG_Z_no_payload
    global gripper_COG_Y_no_payload
    
    if with_weight == True:
        payload= gripper_weight_with_payload
        COG_Y= gripper_COG_Y_with_payload
        COG_Z= gripper_COG_Z_with_payload
    elif with_weight == False:
        payload= gripper_weight_no_payload
        COG_Y= gripper_COG_Y_no_payload
        COG_Z= gripper_COG_Z_no_payload
    
    send_data= "(" + str(payload) + "," + str(COG_Y)  + "," + str(COG_Z)  +", 0, 0, 0, 9, 0,0,0,0,0,0,0)"
    print "send data: " +  send_data
    c.send(send_data)
    msg=""
    while (msg != "OK"):
      		msg = c.recv(2)

def fine_movement_for_suction_checking(axis,threshold,step,speed,bin_number):
    global count_fine_movement
    global step_fine_movement
    global axis_fine_movement
    sucking=False
    load = 0
    axis_fine_movement=axis
    step_fine_movement=step
    count_fine_movement=0
    sucking=check_suction_status()
    load = Ave_force()
    threshold = 27
    if bin_number == 3:
        threshold = 35

    while (sucking ==False and count_fine_movement <= threshold and load <threshold ):
        count_fine_movement +=1
        move_end_effector(axis, step, speed)
        sucking=check_suction_status()
        load = Ave_force()
        print "Load: " + str(load)


def order_bin(bin_num):

    order_pose=[[0.05,-0.05],[0.05,0],[0.05,0.05],[0.1,-0.05],[0.1,0],[0.1,0.05]]
    if bin_num>=1 and bin_num<=6:
            num=bin_num-1
    elif bin_num>=7 and bin_num<=12:
            num=bin_num-7
    
    
    distance_z=order_pose[num][0]
    distance_x=order_pose[num][1]
    move_end_effector_2_axis_translation('z', distance_z, 0.1, 'x', distance_x)
    '''
    axis='z'
    move_end_effector(axis, distance_z, 0.05)
    axis='x'
    move_end_effector(axis, distance_x, 0.05)
    '''
    
def check_suction_status():
    c.send(b"(0, 0, 0, 0, 0, 0, 6, 0,0,0,0,0,0,0)")
    msg = ""
    while (msg != "Y" and msg != "N"  ):
        msg = c.recv(1)
    
    if msg== "Y":
        return True
    elif msg== "N":
        return False
        
	
def suction(state):
    global suction_method
    if (state == True):
	    if suction_method == 2 or suction_method ==5 or suction_method ==8 :
		    c.send(b"(0, 0, 0, 0, 0, 0, 3,0,0,0,0,0,0,0)")
	    elif suction_method == 1 or suction_method ==6 or suction_method ==7:
		    c.send(b"(0, 0, 0, 0, 0, 0, 4,0,0,0,0,0,0,0)")
    elif (state == False):
        c.send(b"(0, 0, 0, 0, 0, 0, 7,0,0,0,0,0,0,0)")


def Move_to_grasping_pose(bin_number,target_pose):
    global suction_method
    global current_pose
    global safety_per
    global EE_move_forward_distance_front_suction
    print "Move to grasping pose now."
    print "The relative grasping pose: " + str(target_pose)
    permission=1
    
    if permission==1:
	    if suction_method!=5:
	        if bin_number == 11:
	            move_end_effector('y', target_pose[1], 0.05)
	            #move_end_effector('x', target_pose[0], 0.05)
	        else:
	            move_end_effector_2_axis_translation('x', target_pose[0], 0.05, 'y', target_pose[1])
	    elif suction_method==5:
	        if (bin_number == 1 or bin_number == 4 or bin_number == 11) and target_pose[5]>0:
	            move_end_effector('x', 0.22, 0.2)
	            
	        move_end_effector('rz', math.degrees(target_pose[5]), 0.2)
	        
	        if (bin_number == 1 or bin_number == 4 or bin_number == 11) and target_pose[5]>0:
	            move_end_effector('y', 0.22, 0.2)
	        if target_pose[5]>= 0:
	            move_end_effector_2_axis_translation('y', -1*target_pose[0], 0.05, 'x', target_pose[1])
	        elif target_pose[5]< 0:
	            move_end_effector_2_axis_translation('y', target_pose[0], 0.05, 'x', -1*target_pose[1])


	    if suction_method==1 or suction_method ==3 or suction_method ==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:   # for items except bottom suck
	        if bin_number == 11:
	            move_end_effector('z', (0.06), 0.05) ## go to item
	            move_end_effector('x', target_pose[0], 0.05)
	            move_end_effector('z', (target_pose[2]-0.02-0.06), 0.05) ## go to item
	        else:
	            move_end_effector('z', (target_pose[2]-0.02), 0.05) ## go to item
	    elif suction_method ==2 or suction_method ==5 or suction_method ==8 :
	        if bin_number == 11 and suction_method != 5:
	            move_end_effector('z', (0.06), 0.05) ## go to item
	            move_end_effector('x', target_pose[0], 0.05)
	            move_end_effector('z', (target_pose[2]-0.06), 0.05) ## go to item
	        else:
	            move_end_effector('z', (target_pose[2]), 0.05) ## go to item
	        
	    
	    move_end_effector('ry', math.degrees(target_pose[4]), 0.05)
	    
	    if suction_method ==8 :# thin item
	        move_end_effector('rx', -4, 0.05) ## rotate down 4 degree
	        move_end_effector('y', (0.03), 0.05) ## go down 3cm
	    if suction_method==1 or suction_method ==3 or suction_method ==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
	        move_end_effector('z', (0.02), 0.05) ## go to item
	    
	    if suction_method==1 or suction_method==3 or suction_method==4 or suction_method ==6 or suction_method ==7 or suction_method ==10 or  suction_method ==11:
	        move_end_effector('rx', -2, 0.05)
	    
	    if suction_method == 1 or suction_method == 6 or suction_method == 7 :
	        z_incre_step = EE_move_forward_distance_front_suction // 0.005
	        fine_movement_for_suction_checking('z',z_incre_step, 0.005,0.02,bin_number)
	        #EE_move_forward_distance_front_suction = 5 * 0.005
	    elif suction_method == 2:
	        #y_incre_step = (((math.fabs(target_pose[1] - target_pose[7]))-0.05) // 0.01) + 5
	        y_incre_step = (math.fabs(target_pose[1] -0.05 + 0.006) // 0.01)
	        print "pose 1: " + str(target_pose[1])
	        print "pose 7: " + str(target_pose[7])
	        print "y incre:" + str(y_incre_step)
	        fine_movement_for_suction_checking('y',y_incre_step, 0.01,0.05,bin_number)
	    elif suction_method == 5:
	        #x_incre_step = (((math.fabs(target_pose[0] - target_pose[6]))-0.05) // 0.01) + 3
	        if bin_number == 2 or bin_number ==5 or bin_number ==8 or bin_number ==11:
	            wall_distance = 0.3/2
	        else:
	            wall_distance = 0.25/2
	        #x_incre_step = (((math.fabs(target_pose[0] - wall_distance))-0.05) // 0.01) + 3
	        if target_pose[0] >=0:
	            if target_pose[5] >=0:
	                x_incre_step = ((math.fabs(target_pose[0]) + wall_distance)-0.05) // 0.01 
	            else:
	                x_incre_step = ((wall_distance - math.fabs(target_pose[0]) )-0.05) // 0.01
	        elif target_pose[0] < 0:
	            if target_pose[5] >=0:
	                x_incre_step = ((wall_distance - math.fabs(target_pose[0]) )-0.05) // 0.01
	            else:
	                x_incre_step = ((wall_distance + math.fabs(target_pose[0]) )-0.05) // 0.01
	        print "pose 0: " + str(target_pose[0])
	        print "pose 6: " + str(target_pose[6]) 
	        print "x incre:" + str(x_incre_step)
	        fine_movement_for_suction_checking('y',x_incre_step, 0.01,0.05,bin_number)
	    elif suction_method==8:
	        #y_incre_step = (((math.fabs(target_pose[1] - target_pose[7]))-0.05-0.03) // 0.01) + 2
	        y_incre_step = (math.fabs(target_pose[1] -0.05+0.006) // 0.01)
	        print "pose 1: " + str(target_pose[1])
	        print "pose 7: " + str(target_pose[7])
	        print "y incre:" + str(y_incre_step)
	        fine_movement_for_suction_checking('y',y_incre_step, 0.01,0.05,bin_number)
	    safety_per=True
	        

 
    else:
        safety_per=False
    
    	
def move_to_item(bin_number,target_pose):
    print "Move to item now."
    print "Received relative target pose: " + str(target_pose)
    global current_pose
    
    move_end_effector('ry', math.degrees(target_pose[4]), 0.05)
    move_end_effector('x', target_pose[0], 0.05)
    move_end_effector('y', target_pose[1], 0.05)


def move_end_effector_2_axis_translation(axis_1, value_1, speed, axis_2, value_2):

    axis = [axis_1, axis_2]
    value = [value_1, value_2]
    x_send = ['0','0']
    y_send = ['0','0']
    z_send = ['0','0']

    for i in range (2):
        if axis[i] == 'x':
            x_send[i] = str(value[i])
        elif axis[i] == 'y':
            y_send[i] = str(value[i])
        elif axis[i] == 'z':
            z_send[i] = str(value[i])

       
    send_data="( " + x_send[0] + "," + y_send[0]  + "," + z_send[0] + ", 0 ,0, 0, 10, " + str(speed) + "," + x_send[1] + "," + y_send[1]  + "," + z_send[1] + ",0,0,0)"
    c.send(send_data)
    state_check()    

def move_end_effector(axis, value, speed):
    x='0'
    y='0'
    z='0'
    rpy_x="0"
    rpy_y="0"
    rpy_z="0"
    if axis == 'rx' or axis=='ry' or axis=='rz':
        radian=math.radians(value)
    
    if axis == 'x':
        x=str(value)
    elif axis == 'y':
        y=str(value)
    elif axis == 'z':
        z=str(value)  
    elif axis=="rx":
        rpy_x=str(radian)
    elif axis=="ry":
        rpy_y=str(radian)
    elif axis=="rz":
        rpy_z=str(radian)
    send_data="( " + x + "," + y  + "," + z + "," + rpy_x + "," + rpy_y + "," + rpy_z + ", 5, " + str(speed) + ",0,0,0,0,0,0)"
    c.send(send_data);
    state_check()
    
##################################################################################
##################################################################################
    
    
#############################################################################################################
#################################--- Collision Avoidance----#################################################
def collision_check(bin_num,target_pose,method):
	# method 1:front 2:bottom 3:big grasp 4:small grasp 5:side 6:front+big 8:bottom but lower item
	
	global current_end_effector_position
	global init_position_EE_x
	global init_position_EE_y
	global init_position_EE_z
	global EE_move_up_distance
	global EE_side_suction_move_up
	global EE_move_forward_distance_front_suction
	
	safety_buffer=0.005
	additional_white_foam = 0.005
	
	Z_limit=0.2
	Z_limit_lower = 0
	
	    

	if bin_num==1 or bin_num==3 or bin_num==10 or bin_num==12:
	    bin_limit_x=0.25
	    bin_limit_y=0.225
	    if bin_num==1 or bin_num==3:
	        Z_limit=0.16
	elif bin_num==2 or bin_num==11:
	    bin_limit_x=0.3
	    bin_limit_y=0.225
	elif bin_num==4 or bin_num==6 or bin_num==7 or bin_num==9:
	    bin_limit_x=0.25
	    bin_limit_y=0.188
	elif bin_num==5 or bin_num==8:
	    bin_limit_x=0.3
	    bin_limit_y=0.188
	    Z_limit=0.25
	
	if method == 1:  
	    bounding_box_left=0.045 + safety_buffer  + additional_white_foam
	    bounding_box_right=0.045 + safety_buffer + additional_white_foam
	    bounding_box_up=0.075 + safety_buffer + (-1*EE_move_up_distance)
	    bounding_box_down=0.045 + safety_buffer
	    Z_limit=Z_limit - EE_move_forward_distance_front_suction
	    
	elif method ==2 or method ==8:
	    bounding_box_left=0.045 + safety_buffer + additional_white_foam
	    bounding_box_right=0.045 + safety_buffer + additional_white_foam
	    bounding_box_up=0.05 + safety_buffer
	    bounding_box_down=0.045 + safety_buffer
	    Z_limit_lower = 0.05
	    
	elif method ==5 :
	    bounding_box_up=0.05 + safety_buffer
	    bounding_box_down=0.045 + safety_buffer
	    if target_pose[5] >0:
	        bounding_box_left=0.045 + safety_buffer + EE_side_suction_move_up + additional_white_foam
	        bounding_box_right=0.045 + safety_buffer + additional_white_foam
	    elif target_pose[5] <0:
	        bounding_box_left=0.045 + safety_buffer + additional_white_foam
	        bounding_box_right=0.045 + safety_buffer + EE_side_suction_move_up + additional_white_foam
	    
	elif method == 3 or method ==7 or method ==10:
	    bounding_box_left=0.09 + safety_buffer + additional_white_foam
	    bounding_box_right=bounding_box_left + additional_white_foam
	    bounding_box_up=0.07 + safety_buffer + (-1*EE_move_up_distance)
	    bounding_box_down=0.06 + safety_buffer
	    if method == 7:
	        Z_limit=Z_limit - EE_move_forward_distance_front_suction
	    
	elif method == 4 or method == 6 :
	    bounding_box_left=0.045 + safety_buffer + additional_white_foam
	    bounding_box_right=bounding_box_left + additional_white_foam
	    bounding_box_up=0.07 + safety_buffer + (-1*EE_move_up_distance)
	    bounding_box_down=0.06 + safety_buffer
	    if method == 6 :
	        Z_limit=Z_limit - EE_move_forward_distance_front_suction
	    
	elif method == 11 :
	    bounding_box_left=0.01 + safety_buffer + additional_white_foam
	    bounding_box_right=bounding_box_left + additional_white_foam
	    bounding_box_up=0.07 + safety_buffer + (-1*EE_move_up_distance)
	    bounding_box_down=0.035 + safety_buffer
	    Z_limit=0.1

	
	approval=False
	
	
	global_x = current_end_effector_position[0] + target_pose[0]
	global_y = current_end_effector_position[1] + target_pose[1]
	global_z = current_end_effector_position[2] + target_pose[2]
	
	X_limit_upper = bin_limit_x/2
	X_limit_lower = -bin_limit_x/2
	Y_limit_upper = -bin_limit_y/2
	Y_limit_lower = bin_limit_y/2
	
	Ry_angle_limit_up = math.radians(10+1)
	Ry_angle_limit_down = math.radians(-10-1)
	Rz_angle_limit_up = math.radians(90+1)
	Rz_angle_limit_down = math.radians(-91-1)
	
	x_axis_left = global_x - bounding_box_left
	x_axis_right = global_x + bounding_box_right
	y_axis_up = global_y - bounding_box_up
	y_axis_down = global_y + bounding_box_down
	z_axis = global_z
	
	##########--Calcilation of rotation y--#########################
	if method == 1 or method == 2 or method == 5 or method == 8:
	    ry_diagonal_L1= math.sqrt(math.pow(bounding_box_right,2) + math.pow((0.045),2))
	    ry_diagonal_L2= math.sqrt(math.pow(bounding_box_right,2) + math.pow((z_axis),2))
	    ry_diagonal_L3= math.sqrt(math.pow(bounding_box_left,2) + math.pow((0.045),2))
	    ry_diagonal_L4= math.sqrt(math.pow(bounding_box_left,2) + math.pow((z_axis),2))
	else:
	    ry_diagonal_L1= math.sqrt(math.pow(bounding_box_right,2) + math.pow((z_axis),2))
	    ry_diagonal_L2= math.sqrt(math.pow(bounding_box_left,2) + math.pow((z_axis),2))

	
	if target_pose[4] >=0 :
	    if method == 1 or method == 2 or method == 5 or method == 8:
	        ry_bounding_box_right = ry_diagonal_L1 * math.cos(math.radians(45) - target_pose[4])
	        ry_bounding_box_left = ry_diagonal_L4 * math.cos(math.radians(45) - target_pose[4])
	    else:
	        ry_bounding_box_right = bounding_box_right
	        ry_bounding_box_left = ry_diagonal_L2 * math.cos(math.radians(45) - target_pose[4])
	elif target_pose[4] <0 :
	    if method == 1 or method == 2 or method == 5 or method == 8:
	        ry_bounding_box_right = ry_diagonal_L2 * math.cos(math.radians(45) - target_pose[4])
	        ry_bounding_box_left = ry_diagonal_L3 * math.cos(math.radians(45) - target_pose[4])
	    else:
	        ry_bounding_box_right = ry_diagonal_L1 * math.cos(math.radians(45) + target_pose[4])
	        ry_bounding_box_left = bounding_box_left
	    
	ry_right = global_x + ry_bounding_box_right
	ry_left = global_x - ry_bounding_box_left
    #################################################################
    
	##########--Calcilation of rotation z meant for method 5 side sucking--#########################
	if method == 5:
	    rz_diagonal_L1= math.sqrt(math.pow(bounding_box_right,2) + math.pow(bounding_box_up,2))
	    rz_diagonal_L2= math.sqrt(math.pow(bounding_box_right,2) + math.pow(bounding_box_down,2))
	    rz_diagonal_L3= math.sqrt(math.pow(bounding_box_left,2) + math.pow(bounding_box_up,2))
	    rz_diagonal_L4= math.sqrt(math.pow(bounding_box_left,2) + math.pow(bounding_box_down,2))
	    if target_pose[5] >=0 :
	        if target_pose[5] >= math.radians(80):
	            rz_bounding_box_right = bounding_box_up
	            rz_bounding_box_left = bounding_box_down
	            rz_bounding_box_up = bounding_box_left
	            rz_bounding_box_down = bounding_box_right
	            #print "b d:" + str(rz_bounding_box_down)
	        else:
	            rz_bounding_box_right = rz_diagonal_L1 * math.cos(math.radians(45) - target_pose[5])
	            rz_bounding_box_left = rz_diagonal_L4 * math.cos(math.radians(45) - target_pose[5])
	            rz_bounding_box_up = rz_diagonal_L3 * math.cos(math.radians(45) - target_pose[5])
	            rz_bounding_box_down = rz_diagonal_L2 * math.cos(math.radians(45) - target_pose[5])
	    elif target_pose[5] <0 :
	        if target_pose[5] <= math.radians(-80):
	            rz_bounding_box_right = bounding_box_down
	            rz_bounding_box_left = bounding_box_up
	            rz_bounding_box_up = bounding_box_right
	            rz_bounding_box_down = bounding_box_left
	        else:
	            rz_bounding_box_right = rz_diagonal_L2 * math.cos(math.radians(45) + target_pose[5])
	            rz_bounding_box_left = rz_diagonal_L3 * math.cos(math.radians(45) + target_pose[5])
	            rz_bounding_box_up = rz_diagonal_L1 * math.cos(math.radians(45) + target_pose[5])
	            rz_bounding_box_down = rz_diagonal_L4 * math.cos(math.radians(45) + target_pose[5])
	        
	    rz_right = global_x + rz_bounding_box_right
	    rz_left = global_x - rz_bounding_box_left
	    rz_up = global_y - rz_bounding_box_up
	    rz_down = global_y + rz_bounding_box_down
	    
	    x_axis_right = rz_right
	    x_axis_left = rz_left
	    y_axis_down = rz_down
	    y_axis_up = rz_up
	    

    #################################################################################################
	
	if (x_axis_right < X_limit_upper) and (x_axis_left > X_limit_lower):
	    if (y_axis_down < Y_limit_lower) and (y_axis_up > Y_limit_upper):
	        if (z_axis >= Z_limit_lower) and (z_axis <= Z_limit):
	            if method == 5:
	                if target_pose[5] < Rz_angle_limit_up and target_pose[5] > Rz_angle_limit_down:
	                    approval=True
	                    print "No collison."
	                else:
	                    approval=False
	                    print "Rz rotation angle out of limit."
	                    print "Current value rotation angle: " + str(target_pose[5])
	                
	            else:
	                if method == 2 or method == 3 or method == 7 or method == 8 or method == 10:
	                    approval=True
	                    print "No collison."
	                elif (ry_right < X_limit_upper) and (ry_left > X_limit_lower):
	                    if target_pose[4] < Ry_angle_limit_up and target_pose[4] > Ry_angle_limit_down:
	                        approval=True
	                        print "No collison."
	                    else:
	                        approval=False
	                        print "Ry rotation angle out of limit."
	                        print "Current value rotation angle: " + str(target_pose[4])
	                else:
	                    approval=False
	                    print "Ry rotation out of limit."
	                    print "Lower limit: " + str (X_limit_lower)
	                    print "Upper limit: " + str (X_limit_upper)
	                    print "Current value x right: " + str(ry_right)
	                    print "Current value x left: " + str(ry_left)
	        else:
	            approval=False
	            print "Z axis out of limit."
	            print "Upper limit: " + str (Z_limit)
	            print "Lower limit: " + str (Z_limit_lower)
	            print "Current value: " + str(z_axis)
	    else:
	        approval=False
	        print "Y axis out of limit."
	        print "Lower limit: " + str (Y_limit_lower)
	        print "Upper limit: " + str (Y_limit_upper)
	        print "Current value Y up: " + str(y_axis_up)
	        print "Current value Y down: " + str(y_axis_down)
	else:
	    approval=False
	    print "X axis out of limit."
	    print "Lower limit: " + str (X_limit_lower)
	    print "Upper limit: " + str (X_limit_upper)
	    print "Current value X right: " + str(x_axis_right)
	    print "Current value X left: " + str(x_axis_left)
	
	#approval = True
	return approval
	

    
##################################################################################
##################################################################################

    
        
#############################################################################################################
#########################################--- Testing----#####################################################
def testing_bin_7_10():
    bin_num=input("Key in the bin number (7 or 10).")
    if (bin_num ==7) or  (bin_num==10) :
        go_to_capturing_position(bin_num,False)
        
        '''
        x=input("Key in the x .")
        move_end_effector('x', 0.02, 0.2)
        x=input("Key in the x .")
        move_end_effector('y', 0.02, 0.2)
        x=input("Key in the x .")
        move_end_effector('z', 0.02, 0.2)
        '''

def testing_go_to_bin():
    bin_num=input("Key in the bin number (1 ~12).")
    if (bin_num >=1) and  (bin_num<=12) :
        pick_n_place(bin_num,3,[0])
        go_to_capturing_position(bin_num,False)
        Ave_force()
        
        '''
        if bin_num ==7 or bin_num ==10:
            c.send(Q_order_bin_7_10)
        else:
            c.send(Q_order_down)
            state_check()
            c.send(Q_order_1)
        state_check()
        order_bin(bin_num)
        move_end_effector('y', 0.2, 0.5)
        move_end_effector('y', -0.2, 0.5)
        #pick_n_place(bin_num,3,[0])
        '''
        '''
        pick_n_place(bin_num,3,[0])
        pick_n_place(bin_num,1,[0])
        
        pick_n_place(bin_num,3,[0])
        c.send(Q_order_down)
        state_check()
        c.send(Q_order_1)
        state_check()
        order_bin(bin_num)
        move_end_effector('y', 0.3, 0.25)
        move_end_effector('y', -0.3, 0.25)
        pick_n_place(bin_num,3,[0])
        '''
    else: print "wrong bin number!"
    
def testing_collision():
    '''
    x=input("Key in the x .")
    y=input("Key in the y .")
    z=input("Key in the z .")
    ry=input("Key in the ry .")
    '''
    collision_check(8,[0.05974,-0.088169,0.25569,0.141,-0.57,0.169],4)
    
##################################################################################
##################################################################################





#######################################################################################
#####################-----------Main Program-------------##############################
#######################################################################################

if __name__ == '__main__': 
	rospy.init_node('Motion_Module', anonymous=True)
	
	for i in range (1,13):
	    Q_bin_new[i] = change_init_pose ((rospy.get_param('motion_module/point/Q' + str(i))),i)
	
	    

		

	
	    
	
	
	while not rospy.is_shutdown():
	    #testing_bin_7_10()
	    #testing_go_to_bin()
	    #testing_collision()
	    service_server()
	 
        




                
	    

                
	    




       



