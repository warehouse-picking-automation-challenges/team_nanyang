#!/usr/bin/env python

import socket
import time
import rospy
import roslib
import sys
import math
import os
import yaml
import csv


from std_msgs.msg import String
from program_manager.srv import *
from identification.srv import *
from RGBD_Capture.srv import *
from registration.srv import *
from mantis.srv import *

#######################################################################################
##############------------------Setting-----------------------------###################
#######################################################################################
initialization_mode=True
Testing_with_Teck_Hou=False

offset_x = 0#0.02
offset_y = 0#0.02
offset_z = 0.015#0.02#0.06#0.04


text_file = open("/home/rrcwp5/NTU-APC/backup29/src/program_manager/src/confidence log.txt", "a")
#text_file.write('.')
#text_file.close()

#######################################################################################
###########-----------Port Defining for Task Planner-----------########################
#######################################################################################
HOST = "192.168.0.104"#"155.69.132.13"#"192.168.0.102"  #"172.22.135.106"#"192.168.0.102"      # The remote host
PORT = 30004            # The same port as used by the server

print "\n#########################################################################"
print "#######################---APC Team Nanyang---############################"
print "#########################################################################\n"
print "Program Manager is started."
print "Waiting for Task Planner to connect."
print "IP               : " + HOST
print "Port number      : " + str(PORT)

s = socket.socket()         # Create a socket object 
s.bind((HOST, PORT))        # Bind to the port 
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.  
time.sleep(1)	
print "Successfully connected with Task Planner ! "
print "#########################################################################\n"

#######################################################################################
###############-----------Functions Declared-------------##############################
#######################################################################################


####################################################################################################
#######################--- Getting Identification Coefficient----###################################

def Read_IPPM_CSV(file_address):
    #print "Reading csv file: <" + file_address[file_address.rfind("/")+1:] + ">"
    ippm = []
    with open(file_address, 'rb') as csvfile:
        ippm_reader = csv.reader(csvfile, delimiter=',')
        for i,row in enumerate(ippm_reader):
            ippm.append([])
            for j,el in enumerate(row):
                ippm[i].append(el)
    return ippm
    
def Get_ALL_Items(directory_address):
    #print "Reading directory <" + directory_address[directory_address.rfind("/")+1:] + ">"
    full_items_list = []
    for x in os.walk(directory_address):
        if ((x[0] != "yml_directory") & (x[1] == [])):
            item_name = x[0][x[0].rfind("/")+1:]
            full_items_list.append(item_name)
    full_items_list.sort()
    return full_items_list
    
def Calculate_bin_identification_priors(ippm, bin_contents, work_order, full_items_list):
    bin_id_priors = []
    for i, I in enumerate(bin_contents):
        prob = 1.0
        if work_order[i] == "dummy":
            prob = 0
        else:
            
            idx = full_items_list.index(work_order[i])
            # print("target " + str(i) + " item is " + work_order[i] + " at " + str(idx))
            for j, J in enumerate(bin_contents[i]):
                idx2 = full_items_list.index(bin_contents[i][j])
                # print("bin " + str(i) + " item " + str(j) + " = " + bin_contents[i][j] + " at " + str(idx2))
                prob = prob * float(ippm[idx][idx2])
        bin_id_priors.append(prob)
    return bin_id_priors

def Identification_Coefficient_Calculation():
    global Item_List_Iden_Coeff
    global Target_item_list
    ippm_file = os.path.dirname(os.path.realpath(__file__)) + "/../../identification/params/identification_pp_matrix.csv"
    #print(ippm_file)
    ippm = Read_IPPM_CSV(ippm_file)
    #print str(Item_List_Iden_Coeff)
    #print str (Target_item_list)
    
    yml_library = os.path.dirname(os.path.realpath(__file__)) + "/../../identification/yml_library/yml_library"
    full_items_list = Get_ALL_Items(yml_library)
    bin_id_priors = Calculate_bin_identification_priors(ippm, Item_List_Iden_Coeff, Target_item_list, full_items_list)
    
    
    return bin_id_priors

######################################################################################
######################################################################################



####################################################################################################
#######################--- Calling Service----######################################################

def Grasping_Service_Call(command,item,pose):
    global num_attempt
    successfully_implemented_grasping_method = 0
    item_num=0
    testing,item_num=Item_Assignment(0,item,2)
    rospy.wait_for_service('gripper_comm')
    print "Command sent to grasping: " + str(command)
    print "Item id sent to grasping: " + str(item_num)
    print "Target pose sent to grasping: " + str(pose)
    try:
        service_status = rospy.ServiceProxy('gripper_comm', SRV_Gripper)
        feedback = service_status(command,item_num,pose,num_attempt,successfully_implemented_grasping_method)
        print "Status feedback from grasping: " + str(feedback.status)
        print " Pose from grasping: " + str(feedback.grasping_pose)
        print " Method from grasping: " + str(feedback.method)
        return feedback.status, feedback.grasping_pose,feedback.method
	    
    except rospy.ServiceException, e:

	    return 0, [0,0,0,0,0,0],100

def Registration_Service_Call(blob_index,rotations,capture_folder,object_name):
      
    rospy.wait_for_service('get_pose')
    '''
    print str(blob_index)
    print str(rotations)
    print capture_folder
    print str(object_name)
    '''
    try:
	    service_status = rospy.ServiceProxy('get_pose', SRV_Registration)
	    feedback = service_status(blob_index,rotations,capture_folder,object_name)
	    return feedback.pose, feedback.confidence, True
	    
    except rospy.ServiceException, e:
	    #print "Registration Service call failed: %s"%e
	    
	    return [-1,-1,-1,-1,-1,-1,-1], 0, False

def Capture_Service_Call(item_count,bin_num):
    global kinect_angle_from_capture_module
    item_bin=""
    bin_list=["bin_A","bin_B","bin_C","bin_D","bin_E","bin_F","bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"]
    item_bin=bin_list[bin_num-1]
    #item_bin=bin_list[bin_num-1+6]
    rospy.wait_for_service('receive_image')
    try:
        service_status = rospy.ServiceProxy('receive_image', SRV_Capture)
        feedback = service_status(item_count,item_bin)
        print "kinect angle: " + str(feedback.kinect_angle)
        kinect_angle_from_capture_module = feedback.kinect_angle
        return feedback.succeed, feedback.folder, True
    except rospy.ServiceException, e:
        return False, "Error", False


def Identification_Service_Call(item_count,item_list,image_addr):
    
    print "item count : " + str(item_count)
    print "item list : " + str(item_list)
    print "image addr : " + str(image_addr)  
    rospy.wait_for_service('identify')
    try:
        service_status = rospy.ServiceProxy('identify', SRV_Identification)
        feedback = service_status(image_addr,item_count,item_list)
        return feedback.blob_index, feedback.confidence, feedback.rotations,True
        #print "Identification rotations: " + str(feedback.rotations)
    except rospy.ServiceException, e:
        #print "Identification Service call failed: %s"%e
        return 0,0,[0],False

def UR5_Service_Call(bin_num,action,target_pose,method):
	rospy.wait_for_service('Call_UR5')
	
	try:
	    service_status = rospy.ServiceProxy('Call_UR5', SRV_Motion)
	    feedback = service_status(bin_num,action,target_pose,method)
	    update_current_pose(feedback.pose)
	    return feedback.reply
	except rospy.ServiceException, e:
	    #print "UR5 Service call failed: %s"%e
	    return False
	    
##################################################################################
##################################################################################



################################################################################################
#######################--- UR Functions----#####################################################
def Collision_check(bin_num,target_pose,method): # checking collision
    print "Checking collision ..."
    msg=UR5_Service_Call(str(bin_num),"15",target_pose,method)	
    return msg


def Move_robot_to_init_ER(bin_num): # Error recovery
    global grasping_method
    global Pose_from_grasping
    print "Moving to init now (Error Recovery mode)."
    msg=UR5_Service_Call(str(bin_num),"16",Pose_from_grasping,grasping_method)	
    #return msg

def Move_robot_to_bin_ER(bin_num): # Error recovery
    global grasping_method
    global Pose_from_grasping
    print "Moving to target bin now (Error Recovery mode)."
    msg=UR5_Service_Call(str(bin_num),"14",Pose_from_grasping,grasping_method)	
    #return msg

def Move_robot_from_bin_to_order_bin_3rd(bin_num):
    print "Moving to order bin now (In progress of moving to order bin)."
    msg=UR5_Service_Call(str(bin_num),"13",[0,0,0,0,0,0],100)	
    #return msg

def Move_robot_from_bin_to_order_bin_2nd(bin_num):
    global grasping_method
    print "Moving to init pose now (In progress of moving to order bin)."
    msg=UR5_Service_Call(str(bin_num),"12",[0,0,0,0,0,0],grasping_method)	
    #return msg

def check_suction_status():
    print "Checking suction status."
    global item_sucked
    item_sucked=False
    item_sucked=UR5_Service_Call('5',"11",[0,0,0,0,0,0],100)	
    


def Move_robot_out_of_bin_ER(bin_num,target_pose):	#Error recovery
    print "Moving robot out of bin now (Error Recovery mode)."
    global grasping_method
    msg=UR5_Service_Call(str(bin_num),"10",target_pose,grasping_method)	
    #return msg


def Move_robot_from_order_bin_to_init(bin_num):
    print "Moving to init pose from order bin now."
    msg=UR5_Service_Call(str(bin_num),"9",[0,0,0,0,0,0],100)	
    #return msg

def Move_robot_from_bin_to_order_bin_1st(bin_num,target_pose):
    print "Moving out of bin now (In progress of moving to order bin)."
    global grasping_method
    msg=UR5_Service_Call(str(bin_num),"8",target_pose,grasping_method)	
    #return msg

def Suction_control(mode):
    print "Turning on/off of suction."
    global grasping_method
    if mode ==1:
        msg=UR5_Service_Call('0',"6",[0,0,0,0,0,0],grasping_method)	
        #return msg
    elif mode ==0:
        msg=UR5_Service_Call('0',"7",[0,0,0,0,0,0],100)	
        #return msg
        

def Move_robot_to_grasping_pose(num,target_pose):
    print "Moving robot to grasping pose now."
    global grasping_method

    msg=UR5_Service_Call(str(num),"5",target_pose,grasping_method)	
    return msg


def Move_robot_to_init_pose():
    print "Moving robot to init pose now"
    msg=UR5_Service_Call("1","3",[0,0,0,0,0,0],100)	
    #return msg

def Move_robot_to_item(num,target_pose):
    print "Moving robot to the item now."
    msg=UR5_Service_Call(str(num),"2",target_pose,100)	
    #return msg

def Move_robot_to_target_bin(num):
	print 'Moving robot to target bin ' + str(num) + "."
	target_pose=[0,0,0,0,0,0]
	msg=UR5_Service_Call(str(num),'1',target_pose,100)
	return msg


def update_current_pose(pose_msg):
	global current_pose	
	pose=[ 0, 0, 0, 0, 0, 0]	
	pose=convert_string_to_pose(pose_msg)
	current_pose=pose

def convert_string_to_pose(msg):
	pose=[ 0, 0, 0, 0, 0, 0]
	pose_test=[ '','' ,'' ,'' ,'' ,'' ]
	test,test_pose=msg.split('[')
	
	msg,test_pose2=test_pose.split(']')
	
	pose_test[0],pose_test[1],pose_test[2],pose_test[3],pose_test[4],pose_test[5]=msg.split(',')
	for i in range (6):
		pose[i]=float(pose_test[i])
	
	return pose


##################################################################################
##################################################################################



#################################################################################################
#######################--- Task Planner Functions----############################################
		
def Analyse_Bin_Task_Planner(data):
	bin_name_list=['A','B','C','D','E','F','G','H','I','J','K','L']
	
	for i in range (12):
	    if (data == bin_name_list[i]):
	        if(i>=0) and (i<=11):
	            status=Move_robot_to_target_bin(i+1)
	            return (i+1),status
	            break
	        else:
	            status=Move_robot_to_target_bin(5)
	            return (5),status
	            break
                
	            
	if data == "#":
	    Shutdown_Connection()
	else:
	    print 'Invalid Command!!'
	    c.send('SKIP')
	    
	    
	return 0,False
	           
	
def Shutdown_Connection():
	data=''
	while (data !='SHUTDOWN'):	    	
		data = c.recv(8)
	c.close()
	print 'Shutdown Connection with Task Planner Successfully!' 
	rospy.signal_shutdown("All Done")


def item_data_update(item,item_count):
    test_0=""
    test_2=""
    test_1=""
    item_test=[0,0,0]

    test_0,test_1=item.split('{')
    test_2,test_0=test_1.split('}')
    
    if (item_count == 1):
        #############################
        testing1,testing=test_2.split(',')
        #############################item_test[0]=int(test_2)
        item_test[0]=int(testing1)
        item_test[1]=0
        item_test[2]=0
		
    elif (item_count == 2):
        a=""
        b=""
        #a,b=test_2.split(',')
        a,b,c=test_2.split(',')
        item_test[0]=int(a)
        item_test[1]=int(b)
        item_test[2]=0
		
    elif (item_count == 3):
        a=""
        b=""
        c=""
        #a,b,c=test_2.split(',')
        a,b,c,d=test_2.split(',')
        item_test[0]=int(a)
        item_test[1]=int(b)
        item_test[2]=int(c)
		
    return item_test
		

def Receive_Data_Task_Planner():


    print "\n#########################################################################"
    print "##############---Receiving Task from Task Planner---#####################"
    print "#########################################################################\n"
    
    data=''
    reply=''
    data_item_count=''
    data_item_raw=''
    data_item=["","",""]
    target_bin=""
    target_item=0
    while (data ==''):
        data = c.recv(256)
    print "Received data from task planner: ", data
    reply='ACK'
    c.send(reply)
    target_bin,target_item=Data_Analysis(data)
    item_name,testing=Item_Assignment(int(target_item),"",1)
    print "Target_bin: ", target_bin
    text_file.write("Target_bin: " + target_bin + "\n")
    print "Target_item: ", item_name
    text_file.write("Target_item: " + item_name + "\n")
    
    return target_bin, target_item
    
def Data_Analysis(data):
    global num_attempt
    text_0=""
    text_1=""
    text_2=""
    tag=""
    content=""
    bin_num=""
    item_count=""
    item=""
    target_item=""
    item_list=["","",""]
    approval=False
    
    text_0,text_1=data.split('[')
    text_2,text_0=text_1.split(']')
    tag=text_2
    text_0=""
    text_1=""
    text_2=""
    text_0,text_1=data.split('(')
    text_2,text_0=text_1.split(')')
    content=text_2
    
    if (tag=="Target"):
        bin_num,target_item, num_attempt_raw=content.split(',')
        num_attempt = int(num_attempt_raw)
        print "Current number of attempt: " + str(num_attempt)
        return bin_num, target_item
    elif (tag=="Bin"):
        temp=content
        text_1,text_0=temp.split('{')
        item_count,text_0=text_1.split(',')
        item_list=item_data_update(content,int(item_count))
        return int(item_count),item_list
    elif (tag=="Confirmation"):
        if (content=="GO"):
            approval=True
        elif (content=="NO-GO"):
            approval=False
        return approval
	        
	    
	        
	    
    elif (tag=="Bin_Target"):
        raw_target_item_list=[0,0,0,0,0,0,0,0,0,0,0,0]
        
        raw_target_item_list[0],raw_target_item_list[1],raw_target_item_list[2],raw_target_item_list[3],raw_target_item_list[4],raw_target_item_list[5],raw_target_item_list[6],raw_target_item_list[7],raw_target_item_list[8],raw_target_item_list[9],raw_target_item_list[10],raw_target_item_list[11],test=content.split(',')
        
        return raw_target_item_list

def Send_Coefficient_To_Task_Planner(iden,regis,grasp):
    msg=""
    if iden==0:
        iden=0
    if regis==0:
        regis=0
    if grasp==0:
        grasp=0
    msg = "[Confidences](" + str(iden) + "," + str(regis) + "," + str(grasp) +")"
    c.send(msg)
    print "I have sent " + msg + " to Task Planner.\n"
    text_file.write("[Confidences](" + str(iden) + "," + str(regis) + "," + str(grasp) +") \n \n")
    
    
    ####################################
    ###### Dun know why receive a extra dummy
    ####################################
    if Testing_with_Teck_Hou==True:
        data=""
        while (data ==''):
            data = c.recv(1)
    #######################################
    
    if Testing_with_Teck_Hou==True:
        data=""
        while (data!="CK"):
	        data=c.recv(2)
        #print data
    else:
        data=""
        while (data!="ACK"):
	        data=c.recv(3)
        #print data
    

def Get_Confirmation_Task_planner():
    data=''
    reply=''
    approval=False
    
    

    
    ####################################
    ###### Dun know why receive a extra dummy
    ####################################
    if Testing_with_Teck_Hou==True:
        data=""
        while (data ==''):
            data = c.recv(1)
    #######################################

    
    data=""
    while (data ==''):
        data = c.recv(256)
    print "Received data from task planner : ", data
    reply='ACK'
    c.send(reply)
    approval=Data_Analysis(data)
    print "Confirmation from task planner: ", str(approval)
    return approval 	    

def Task_Status_Task_Planner(mode): #1:fail; 2:succeed
	if (mode==1):
	    msg="[Task](NO_GO)"
	elif (mode==2):
	    msg="[Task](GO)"
	c.send("[Task](GO)")  #####   Temporary set the task is success
	data=""
	while (data!="ACK"):
	    data=c.recv(3) 	
		
	print "I have sent " + msg + " to Task Planner.\n"
	
	########################################
	#### dun know y need a dummy receive
	#######################################
	if initialization_mode==True and Testing_with_Teck_Hou==True:
	    data=""
	    while (data ==''):
	        data = c.recv(1)
    #######################################


def Init_Receive_Bins_Item():

    print "-------- Receiving Bin Contents from Task Planner--------"
    global Bin_Content
    global Item_List_Iden_Coeff
    data=""
    while (data !="START_INIT"):
        data = c.recv(10)
    print "Received initialization data from task planner: "  , data
    reply='ACK'
    c.send(reply)
    
    ####################################
    ###### Dun know why receive a extra dummy
    ####################################
    if Testing_with_Teck_Hou==True:
        data=""
        while (data ==''):
            data = c.recv(1)
    #######################################
    
    for i in range(1,13):        ################################################################### is 13,9,7
        data=""
        item_count=0
        item_list=[0,0,0]
        testing_list=[]
        item_name=["","",""]
        while (data ==''):
            data = c.recv(256)
        print "Received initialization data from task planner: "  , data
        reply='ACK'
        c.send(reply)
        
        item_count,item_list=Data_Analysis(data)
        for j in range (3):
            item_name[j],testing=Item_Assignment(item_list[j],"",1)

        print "Bin " + str(i) + ": " 
        print "Item count: " + str(item_count) + " Item: { " + item_name[0] + " , " + item_name[1] + " , " + item_name[2] + " }"
        
        Bin_Content[i][1]=str(item_count)
        Bin_Content[i][2]=item_name[0]
        Bin_Content[i][3]=item_name[1]
        Bin_Content[i][4]=item_name[2]
        
        for i in range (item_count):
            testing_list.append(item_name[i])
            
        Item_List_Iden_Coeff.append(testing_list)
        
    data=""
    while (data!="END_INIT"):
	    data=c.recv(8)
    print "Received initialization data from task planner: "  , data
    reply='ACK'
    c.send(reply) 


def Init_Send_Coeeficient():
    global Target_item_list
    target_num=[0,0,0,0,0,0,0,0,0,0,0,0]
    print "\n-------- Sending Identification Coefficients to Task Planner--------"
    coeff_list=Identification_Coefficient_Calculation()
    
    for i in range (12):
        test,target_num[i]=Item_Assignment(0,Target_item_list[i],2)
    
    
    #print "Identification Coefficient: " + str (coeff_list)
    
    msg_Iden="[Identification](["+ str(target_num[0]) + "," + str(coeff_list[0]) + "], [" +  str(target_num[1]) + "," + str(coeff_list[1]) + "],[" + str(target_num[2]) + "," + str(coeff_list[2]) + "],[" + str(target_num[3]) + "," + str(coeff_list[3] ) + "],[" + str(target_num[4]) + "," + str(coeff_list[4]) + "],[" + str(target_num[5]) + "," + str(coeff_list[5]) + "],[" + str(target_num[6]) + "," +str(coeff_list[6]) + "],[" + str(target_num[7]) + "," + str(coeff_list[7]) + "],[" + str(target_num[8]) + " ,"+ str(coeff_list[8]) + "],[" + str(target_num[9]) + "," + str(coeff_list[9]) + "],[" + str(target_num[10]) + "," + str(coeff_list[10]) + "],[" + str(target_num[11]) + "," + str(coeff_list[11])  + "])"
    print "Identification Coefficient msg sent: " + msg_Iden
    #msg_Iden="[Identification]([159,0.8],[160,0.9],[161,0.8],[162,0.9],[163,0.8],[164,0.9],[165,0.8],[166,0.9],[167,0.8],[168,0.9],[169,0.8],[170,0.9])"
    
    c.send(msg_Iden)
    
    data=""
    while (data!="ACK"):
	    data=c.recv(3)
	    
    print "Identification Message sent!\n"
    	
	#########################################
	##### dUN KNOW WHY NEED A EXTRA DUMMY
	########################################
    if Testing_with_Teck_Hou == True:
	    data=""
	    while (data ==''):
	        data = c.recv(1)
    #######################################


def Init_Receive_Target_Item():

    print "\n-------- Receiving Target items from Task Planner--------"
    global Target_item_list
    
    data=""
    while (data!="START_TARGET"):
	    data=c.recv(12)
    print "Received initialization data from task planner: "  , data
    reply='ACK'
    c.send(reply) 
    
    
    #########################################
	##### dUN KNOW WHY NEED A EXTRA DUMMY
	########################################
    if Testing_with_Teck_Hou == True:
	    data=""
	    while (data ==''):
	        data = c.recv(1)
    #######################################
    
    
    data=""
    while (data==""):
	    data=c.recv(256)
    
    reply='ACK'
    print "Received target list: " + data
    c.send(reply)    
    test_list=Data_Analysis(data)
    
    for i in range (0,12):          ################################################################ is 12,8,4
        Target_item_list[i],testing=Item_Assignment(int(test_list[i]),"",1)
        
     
     
    data=""
    while (data!="END_TARGET"):
	    data=c.recv(10)
    print "Received initialization data from task planner: "  , data
    reply='ACK'
    c.send(reply) 
        
  
	
def Initialization_Task_Planner():

    print "\n#########################################################################"
    print "##############---Initialization with Task Planner---#####################"
    print "#########################################################################\n"
    Init_Receive_Bins_Item()
    
    Init_Receive_Target_Item()
    
    Init_Send_Coeeficient()
    

def Item_Assignment(ID,item,mode):  #mode=1, id-->item, mode=2, item-->id

    item_name_list=["paper_mate_12_count_mirado_black_warrior","elmers_washable_no_run_school_glue","laugh_out_loud_joke_book",\
                   "feline_greenies_dental_treats","kong_sitting_frog_dog_toy","genuine_joe_plastic_stir_sticks",\
                   "expo_dry_erase_board_eraser","rolodex_jumbo_pencil_cup","kong_duck_dog_toy","safety_works_safety_glasses",\
                   "mark_twain_huckleberry_finn","first_years_take_and_toss_straw_cup","champion_copper_plus_spark_plug",\
                   "cheezit_big_original","munchkin_white_hot_duck_bath_toy","kyjen_squeakin_eggs_plush_puppies",\
                   "sharpie_accent_tank_style_highlighters","stanley_66_052","kong_air_dog_squeakair_tennis_ball",\
                   "oreo_mega_stuf","crayola_64_ct","mommys_helper_outlet_plugs","highland_6539_self_stick_notes",\
                   "mead_index_cards","dr_browns_bottle_brush"]
    if mode==1:            
        if ID >=159 and ID <=183:
            item= item_name_list[ID-159]
        else:
            item ="dummy"
            
        return item,0
    elif mode==2:
        ID=0
        for i in range (25):             
            if item==item_name_list[i]:
                ID = i+159
                break
                
        return "",ID
    
    

def Update_item_name_list(item_count,item_list):
    global Current_item_name
    
    if item_count==1:
        Current_item_name=[""]
        Current_item_name[0]=item_list[0]
    elif item_count==2:
        Current_item_name=["",""]
        Current_item_name[0]=item_list[0]
        Current_item_name[1]=item_list[1]

def Analyse_Item_Task_Planner(bin_num,target_item):
    item_count=0
    item_name_list=["","",""]
    target_name,testing=Item_Assignment(target_item,"",1)
    
    item_count=int(Bin_Content[bin_num][1])
    item_name_list[0]=target_name
    if (Bin_Content[bin_num][2]==target_name):
        item_name_list[1]=Bin_Content[bin_num][3]
        item_name_list[2]=Bin_Content[bin_num][4]
    elif (Bin_Content[bin_num][2]!=target_name):
        item_name_list[1]=Bin_Content[bin_num][2]
        if (Bin_Content[bin_num][3]!=target_name):
            item_name_list[2]=Bin_Content[bin_num][3]
        else:
            item_name_list[2]=Bin_Content[bin_num][4]

    
    
    return  item_count,item_name_list

##################################################################################
##################################################################################





####################################################################################################################
#######################--Function meant for Grasping module---######################################################
def checking_grasping_status():
	global grasping_method
	global Current_item_name
	
	if grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8:
		check_suction_status()
	if grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11:
		Grasping_Module(3,Current_item_name[0],[0,0,0,0,0,0]) #Sent command to check grasping
	
		
def activation_grasping_or_suction_routine(mode):		#mode=0,turn off/release	; mode=1, turn on/ grasp
	global grasping_method
	global Current_bin_number
	global Pose_from_grasping
	global Current_item_name
	global Safety_Permission
	global item_sucked
	global grasping_status
	
	if mode==1:
		if grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method ==6 or  grasping_method==7 or grasping_method==8:
			print "Turn on suction."
			Suction_control(1)
		print "Current pose from grasping: " + str(Pose_from_grasping)	
		Grasping_Module(grasping_method+40,Current_item_name[0],[0,0,0,0,0,0]) #Sent command to standby
		status=Move_robot_to_grasping_pose(Current_bin_number,Pose_from_grasping)
		############################################################################
		################--meant for safety purpose --###############################
		############################################################################
		if status==False:
			print "Grasping action is cancelled."
			Suction_control(0)
			Safety_Permission=False
		elif status ==True:
		############################################################################
			Safety_Permission=True
			if grasping_method !=6 and grasping_method!=7:
			    Grasping_Module((grasping_method+10),Current_item_name[0],[0,0,0,0,0,0]) #Sent command to grasp/suck item
			if grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method ==6 or  grasping_method==7 or grasping_method==8:
			    check_suction_status()
			if (grasping_method ==6 or grasping_method==7) and item_sucked==True:
			    Grasping_Module((grasping_method+10),Current_item_name[0],[0,0,0,0,0,0]) #Sent command to grasp/suck item
			    check_suction_status()
			elif (grasping_method ==6 or grasping_method==7) and item_sucked==False:
			    grasping_status = False
			    
			    

        	
	elif mode==0:
		if grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or grasping_method==7 or grasping_method==8:
			print "Turn off suction."
			Suction_control(0)
		if grasping_method==3 or grasping_method==4 or grasping_method==6 or grasping_method==7 or grasping_method==10 or grasping_method==11:
			print "Open gripper."
			Grasping_Module(2,Current_item_name[0],[0,0,0,0,0,0]) # Send command to release
			time.sleep(3)
			
			
def Grasp_data_analysis(data):
    global Current_bin_number
    valid=False
    feasible_pose=[0,0,0,0,0,0,0,0,0,0,0,0]
    method=0
    coeficient=0
    pose_list=[[0 for x in range(6)] for x in range(15)]
    coefficient_list=[0 for x in range(15)]
    method_list=[0 for x in range(15)]
    num_solution=0


    
    
    for i in range (15):
        if data[(i*8)]!=0:
            num_solution+=1
            coefficient_list[i]=data[(i*8)+0]
            method_list[i]=data[(i*8)+7]
            for j in range(6):
                pose_list[i][j]=data[(i*8)+j+1]
        else:
            
            break
            
   
    if num_solution >=1:
        ##check collision
        for i in range (num_solution):
            pose=[0,0,0,0,0,0]
            for j in range (6):
                pose[j]=pose_list[i][j]
            ok=Collision_check(Current_bin_number,pose,method_list[i])
            if ok ==True:
                valid=True
            elif ok ==False:
                coefficient_list[i]=0
        if valid == True:
            max_value = max(coefficient_list)
            max_index = coefficient_list.index(max_value)
 
            for j in range (6):
                feasible_pose[j]=pose_list[max_index][j]
            
            method=method_list[max_index]
            coeficient=coefficient_list[max_index]
                    
    else:
        valid=False         
        
    print "--------Activation of Collision Check -------"
    print "Feasible pose: " + str(feasible_pose)
    print "Grasping method: " + str(method)
    print "Grasping coefficient: " + str(coeficient)
    return valid, feasible_pose,method,coeficient

##################################################################################
##################################################################################





####################################################################################################################
#######################--- Pose Calculation (Registration) Functions----############################################

def pose_calculation(pose):
    global Current_bin_number
    global offset_x
    global offset_y
    global offset_z
    global kinect_angle_from_capture_module
    
    new_pose=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    x=pose[0]
    y=pose[1]
    z=pose[2]
    rx=pose[3]
    ry=pose[4]
    rz=pose[5]
    x_shelf=0
    y_shelf=0
    z_shelf=0
    rx_shelf=0
    ry_shelf=0
    rz_shelf=0
    kinect_slanted_angle_x = math.degrees(math.fabs(kinect_angle_from_capture_module[0]))
    kinect_slanted_angle_y = -math.degrees(math.fabs(kinect_angle_from_capture_module[1]))
    distance_kinect_to_end_effector_z_axis = 0.23
    gripper_length=0.36
    distance_kinect_to_end_effector_x_axis = 0.01
    
 
    x_shelf=x + distance_kinect_to_end_effector_x_axis
    y_shelf=-1*(((y*math.cos(math.radians(kinect_slanted_angle_x))) - (z*math.sin(math.radians(kinect_slanted_angle_x))))+ distance_kinect_to_end_effector_z_axis )
    z_shelf=((y*math.sin(math.radians(kinect_slanted_angle_x))) + (z*math.cos(math.radians(kinect_slanted_angle_x))))-gripper_length
    
    rx_shelf=rx
    ry_shelf=(-1)*((ry*math.cos(math.radians(kinect_slanted_angle_x))) - (rz*math.sin(math.radians(kinect_slanted_angle_x))))
    rz_shelf=((ry*math.sin(math.radians(kinect_slanted_angle_x))) + (rz*math.cos(math.radians(kinect_slanted_angle_x))))
    
    ###############################
    
    y_shelf_new = y_shelf
    
    x_shelf_new = ((x_shelf*math.cos(math.radians(kinect_slanted_angle_y))) + (z_shelf*math.sin(math.radians(kinect_slanted_angle_y)))) 
    z_shelf_new = ((x_shelf*math.sin(math.radians(kinect_slanted_angle_y))) + (z_shelf*math.cos(math.radians(kinect_slanted_angle_y))))
    
    ry_shelf_new = ry_shelf
    rx_shelf_new = ((rx_shelf*math.cos(math.radians(kinect_slanted_angle_y))) + (rz_shelf*math.sin(math.radians(kinect_slanted_angle_y)))) 
    rz_shelf_new = ((rx_shelf*math.sin(math.radians(kinect_slanted_angle_y))) + (rz_shelf*math.cos(math.radians(kinect_slanted_angle_y))))
    '''
    print "X new: " + str(x_shelf_new)
    print "Y new: " + str(y_shelf_new)
    print "Z new: " + str(z_shelf_new)
    print "Rx new: " + str(rx_shelf_new)
    print "Ry new: " + str(ry_shelf_new)
    print "Rz new: " + str(rz_shelf_new)
    '''
    ###############################
    
    new_pose[0]=x_shelf_new + offset_x ################################################## OFFSET 2cm TESTING
    new_pose[1]=y_shelf_new + offset_y
    new_pose[2]=z_shelf_new + offset_z  ################################################## OFFSET 4cm TESTING
    new_pose[3]=rx_shelf_new
    new_pose[4]=ry_shelf_new
    new_pose[5]=rz_shelf_new
    new_pose[6]=pose[6]######################
    new_pose[7]=pose[7]########################
    new_pose[8]=pose[8]########################
    new_pose[9]= Current_bin_number
    return new_pose   
    
def Out_of_range_detector(pose):
    if pose[0] <= 0.19 and pose[0] >= -0.19:
       
        if pose[1] >= -0.26 and pose[1] <= 0.08:
            if pose[2] >= 0 and pose[2] <= 0.8:
                return False
            else: return True  
        else: return True
    else: return True

##################################################################################
##################################################################################




####################################################################################################################
#########################################--- Module Calling----#####################################################
def Capture_Module(item,bin_num):

    global capture_status_server
    global capture_status
    global folder_addr_from_capture_module
    
    capture_status_server=False
    capture_status=False
    folder_addr_from_capture_module=''
    
    print "\n-------- Activation of Capture Module--------"
    
    capture_count=0
    while ((capture_status==False or capture_status_server==False) and capture_count<2 ):
	    time.sleep(2)
	    capture_status,folder_addr_from_capture_module,capture_status_server= Capture_Service_Call(item,bin_num)
	    capture_count +=1
    
    print "Capturing status: " + str(capture_status)
    print "Capturing status of server: " + str(capture_status_server)
    print "Capturing folder address: " + folder_addr_from_capture_module
	

def Identification_Module(item_count,item_name,folder):

    global blob_index
    global identification_confidence
    global identification_rotations
    global identification_status
    
    blob_index=0
    identification_confidence=0
    identification_status=False
    identification_rotations=[0 for x in range(45)]
    
    print "\n-------- Activation of Identification Module--------"
	    
    blob_index,identification_confidence,identification_rotations,identification_status=Identification_Service_Call(item_count,item_name,folder )
    
    print "Blob index: " + str(blob_index)
    print "Identification confidence: " + str(identification_confidence)
    print "Identification rotations: " + str(identification_rotations)
    print "Identification status: " + str(identification_status)


def Registration_Module(blob,rotations,folder,current_item):
    global item_pose
    global registration_confidence
    global registration_status
	
    item_pose=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    registration_confidence=0
    registration_status=False
    
    print "\n-------- Activation of Registration Module--------"
    
    item_pose,registration_confidence,registration_status=Registration_Service_Call(blob,rotations,folder,current_item)
    
    print "registration confidence: " + str(registration_confidence)
    print "registration pose: " + str(item_pose)


def Grasping_Module(command,item,pose):
    global grasping_status
    global Pose_from_grasping
    global grasping_method
    global grasping_coeficient
    status_raw=0
    #Pose_from_grasping=[0,0,0,0,0,0]
    grasping_status=False
    #grasping_method=0
    Feasible_solution=False
    pose_data=[0 for x in range(120)]
    
    
    print "\n-------- Activation of Grasping Module--------"
    
    status_raw,pose_data,grasp_method_test=Grasping_Service_Call(command,item,pose)

    if status_raw == 10:
	    print "Gripper in standby position...."
	    grasping_status=True
    elif status_raw == 20:
	    print "Gripping action initiated...."
	    grasping_status=True
    elif status_raw == 30:
	    print "Gripper in released position...."
	    grasping_status=True
    elif status_raw == 40:
	    print "Item still secured...."
	    grasping_status=True
    elif status_raw == 41:
	    print "ITEM DROPPED...."
	    grasping_status=False
    elif status_raw == 42:
	    print "Finger not in used, check vacuum...."
	    grasping_status=True
	    
    if command== 0:
        Feasible_solution,Pose_from_grasping,grasping_method,grasping_coeficient=Grasp_data_analysis(pose_data)
        for i in range (6):
            Pose_from_grasping[i+6] = pose[i]
        if Feasible_solution==False:
            print "No feasible solution or collision occurs."
            grasping_status=False
   
    

def Update_Coeff_Seek_Permission_Task_Planner():
    global identification_confidence
    global registration_confidence
    global grasping_coeficient
    global Approval_Task_Planner
    
    Send_Coefficient_To_Task_Planner(identification_confidence,registration_confidence,grasping_coeficient)
    Approval_Task_Planner=False
    Approval_Task_Planner=Get_Confirmation_Task_planner()
    print "Approval from Task Planner: " + str(Approval_Task_Planner)
    
    

##################################################################################
##################################################################################




####################################################################################################################
#########################################--- ERROR Recovery----#####################################################
def Error_Recovery(mode):
    # mode:
    # 1= motion, 2=capture, 3=identification, 4=registration, 5=registration posture out of range, 6=grasping(pose),7=grasping(grasp/suck) in the bin, 8 = grasping (item dropped in bin) , 9 = grasping (item dropped on ground 1st stage)
    
    print "\n-------- ERROR Recovery Mode--------"
    
    global Current_item_count
    global Current_bin_number
    global capture_status
    global capture_status_server
    global folder_addr_from_capture_module
    global blob_index
    global identification_rotations
    global Current_item_name
    global identification_status
    global registration_status
    global identification_confidence
    global new_target_pose
    global item_pose
    global out_of_range
    global grasping_status
    global grasping_method
    global Pose_from_grasping
    global item_sucked
    grasping_status = False
    item_sucked=False
    print "mode: "+ str(mode)
    
    
    
    if mode==1:
        print "Error in Motion Module."
        Move_robot_to_init_pose()
        Send_Coefficient_To_Task_Planner(0,0,0)
        if Testing_with_Teck_Hou==True:
            testing=Get_Confirmation_Task_planner()
            
    elif mode==2:
        print "Error in Capture Module."
        Move_robot_to_init_pose()
        Send_Coefficient_To_Task_Planner(0,0,0)
        if Testing_with_Teck_Hou==True:
            testing=Get_Confirmation_Task_planner()
            
    elif (mode==3 or mode==4 or mode==5 or mode==7 or mode==8):
        if mode==3:
            print "Error in Identification Module."
        elif mode==4:
            print "Error in Registration Module."
        elif mode==5:
            print "Error in Registration Module, posture out of range."
        elif mode==7:
            print "Error in Grasping Module, fail to grasp/suck item."
            activation_grasping_or_suction_routine(0) # turn off/release@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            Move_robot_out_of_bin_ER(Current_bin_number,Pose_from_grasping)
            
        elif mode==8:
            print "Error in Grasping Module, item dropped in the bin."
            activation_grasping_or_suction_routine(0) # turn off/release
            Move_robot_to_bin_ER(Current_bin_number)
            
           

        Capture_Module(Current_item_count,Current_bin_number)
        
        if (capture_status==False or capture_status_server==False):
            print "***************************************--------Capture module failed!\n"
            Move_robot_to_init_pose()
            Send_Coefficient_To_Task_Planner(0,0,0)
            if Testing_with_Teck_Hou==True:
                testing=Get_Confirmation_Task_planner()
        elif (capture_status==True and capture_status_server==True):
            print "######################################--------Capture module succeed! \n"
            Identification_Module(Current_item_count,Current_item_name,folder_addr_from_capture_module )
            
            if (identification_status==False):
                if mode==3:
                    print "**************************************------Identification module failed twice! \n"
                elif mode==4 or mode==5 or mode==7 or mode==8:
                    print "**************************************------Identification module failed! \n"
                Move_robot_to_init_pose()
                grasping_status = False
                Send_Coefficient_To_Task_Planner(0,0,0)
                if Testing_with_Teck_Hou==True:
                    testing=Get_Confirmation_Task_planner()
            elif ((identification_status==True) and (mode ==4 or mode==5 or mode==7 or mode==8 )):
                print "#####################################------Identification module succeed! \n"
                Registration_Module(blob_index,identification_rotations,folder_addr_from_capture_module,Current_item_name[0])
                
                if (registration_status==False):
                    if mode==4:
                        print "**************************************------Registration module failed twice! \n"
                    elif (mode==5 or mode==7 or mode==8):
                        print "**************************************------Registration module failed! \n"
                    Move_robot_to_init_pose()
                    Send_Coefficient_To_Task_Planner(identification_confidence,0,0)
                    if Testing_with_Teck_Hou==True:
                            testing=Get_Confirmation_Task_planner()
                    
                elif (registration_status==True and (mode==5 or mode==7 or mode==8)):
                    new_target_pose=[0,0,0,0,0,0,0,0,0]
                    new_target_pose=pose_calculation(item_pose)
                    out_of_range=True
                    out_of_range=Out_of_range_detector(new_target_pose)
                    if out_of_range==True:
                        if mode==5:
                            print "Registered Posture is Out of Range twice!!"
                        elif mode==7 or mode==8:
                            print "Registered Posture is Out of Range !!"
                        print "Relative posture: " + str(new_target_pose)
                        print "UR will go back to init position."
                        Move_robot_to_init_pose()
                        Send_Coefficient_To_Task_Planner(0,0,0)
                        if Testing_with_Teck_Hou==True:
                            testing=Get_Confirmation_Task_planner()
                    elif out_of_range==False and (mode ==7 or mode==8):
                        print "Registered posture is feasible!"
                        print "Relative posture: " + str(new_target_pose)
                        Grasping_Module(0,Current_item_name[0],new_target_pose) #get target pose from grasping
                        print "\n-------- Seeking Approval from Task Planner--------"
                        Update_Coeff_Seek_Permission_Task_Planner()
                        if (Approval_Task_Planner == False):
                            print "Grasping action is not allowed from Task Planner!"
                            print "UR will go back to init position."
                            Move_robot_to_init_pose()
                            item_sucked=False
                            grasping_status=False
                        elif (Approval_Task_Planner == True):
                            print "Grasping Module is activated! "
                            print "\n-------- Activation of Grasping module--------"
                            print "Starting Grasping action now !"
                            print "Request for target pose from Grasping Module now !"
                            
                            #permission=input("Grasp?  (1=yes;2=no).")
                            permission =1
                            if permission ==1:
                                
                                
                                if grasping_status==False:
                                    print "#####################################-------Grasping module failed! \n"
                                    print "Fail to get target pose from Grasping Module or collision occurs."
                                    print "UR will go back to init position."
                                    Move_robot_to_init_pose()
                                    Send_Coefficient_To_Task_Planner(0,0,0)
                                    item_sucked=False
                                    grasping_status=False
                                    if Testing_with_Teck_Hou==True:
                                        testing=Get_Confirmation_Task_planner()
                                elif grasping_status==True:
                                    print "Successfully getting the grasping pose."
                                    print "Moving robot to grasping target pose now."
                                    activation_grasping_or_suction_routine(1) # turn on, move to grasping target pose, then grasp
                                    
                                    if Safety_Permission==True:
                                    
                                    	if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or grasping_method==7 or grasping_method==10 or grasping_method==11)  and grasping_status==False) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or grasping_method==7 or grasping_method==8) and item_sucked==False):
                                    		print "#####################################-------Grasping module failed! \n"
                                    		if mode==7:
		                                    	print "Fail to grasp/suck the item twice."
	                                    	elif mode==8:
		                                    	print "Fail to grasp/suck the item."
	                                    	print "I'll get out of the bin and go to init pose."
	                                    	activation_grasping_or_suction_routine(0) # turn off/release
	                                    	Move_robot_out_of_bin_ER(Current_bin_number,Pose_from_grasping)
	                                    	
	                                    	Move_robot_to_init_pose()
	                                    	Send_Coefficient_To_Task_Planner(0,0,0)
	                                    	item_sucked=False
	                                    	grasping_status=False
	                                    	if Testing_with_Teck_Hou==True:
	                                    		testing=Get_Confirmation_Task_planner()
	                                    		
                                    	elif ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==True and mode==8) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8 ) and item_sucked==True and mode==8):
                                    		
                                    		print "Successfully grasp/suck the item."
                                    		print "Moving robot out of bin now."
                                    		Move_robot_from_bin_to_order_bin_1st(Current_bin_number,Pose_from_grasping)
                                    		checking_grasping_status()
                                    		
                                    		if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==False) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8 ) and item_sucked==False):
                                    			print "#####################################-------Grasping module failed! \n"
                                    			print "Item dropped in the bin twice."
                                    			print "UR5 will go to init pose."
                                    			activation_grasping_or_suction_routine(0) # turn off/release
                                    			Move_robot_to_init_ER(Current_bin_number)
                                    			Send_Coefficient_To_Task_Planner(0,0,0)
                                    			item_sucked=False
                                    			grasping_status=False
                                    			if Testing_with_Teck_Hou==True:
                                    				testing=Get_Confirmation_Task_planner()
                                    
                                    elif Safety_Permission==False:
                                    	Move_robot_to_init_pose()
                                    	Send_Coefficient_To_Task_Planner(2,2,2)
                                    	item_sucked=False
                                    	grasping_status=False
                                    	if Testing_with_Teck_Hou==True:
                                    		testing=Get_Confirmation_Task_planner()
                            else:
                                Move_robot_to_init_pose()
                                grasping_method=0
                                Send_Coefficient_To_Task_Planner(2,2,2)
                                if Testing_with_Teck_Hou==True:
                                    testing=Get_Confirmation_Task_planner()
                                
                                    
                                    

    elif mode==6:
        print "Error in Grasping Module, fail to get target pose or collision occurs."
        Grasping_Module(0,Current_item_name[0],new_target_pose)
        if grasping_status == False:
            print "Fail to get target pose or collision occurs happens twice!!"
            print "UR will go back to init position."
            Move_robot_to_init_pose()
            Send_Coefficient_To_Task_Planner(0,0,0)
            if Testing_with_Teck_Hou==True:
                testing=Get_Confirmation_Task_planner()
                
    elif mode==9:
        print "Error in Grasping Module, item dropped on the ground."
        print "This error is non-recoverable."
        print "UR will go back to init position."
        activation_grasping_or_suction_routine(0) # turn off/release
        Move_robot_to_init_pose()
        Send_Coefficient_To_Task_Planner(2,2,2)
        if Testing_with_Teck_Hou==True:
            testing=Get_Confirmation_Task_planner()
        
        
                 
        

   










##################################################################################
##################################################################################




#######################################################################################
#####################-----------Main Program-------------##############################
#######################################################################################

if __name__ == '__main__': 

	current_pose=[0,0,0,0,0,0]	# Will be updated globally automatically everytime after UR5 moving	 
	Bin_Content = [["" for x in range(5)] for x in range(13)]
	
	Target_item_list = ["" for x in range(12)]
	
	Item_List_Iden_Coeff=[]
	
	capture_status_server=False
	capture_status=False
	folder_addr_from_capture_module=""
	blob_index=0
	identification_confidence=0
	identification_rotations=[0 for x in range(45)]
	identification_status=False
	
	item_pose=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	registration_confidence=0
	registration_status=False
	Approval_Task_Planner=False
	grasping_coeficient=0.0
	grasping_status=False
	grasping_method=100
	Pose_from_grasping=[0,0,0,0,0,0,0,0,0,0,0,0]
	kinect_angle_from_capture_module = [0,0,0]
	num_attempt = 0
	
	item_sucked=False
	Safety_Permission=False
	    
	
	rospy.init_node('Program_Manager', anonymous=True)
	
	if initialization_mode==True:
	    Initialization_Task_Planner()


	while not rospy.is_shutdown():
		
		Task_planner_bin_number_raw=""
		Current_item_count=0
		Current_item_name=["","",""]
		Current_target_item_raw=""
		Current_bin_number=0
		UR_moving_status=False
        
        
		#####################################
		####---Calling Task Planner-----#####
		#####################################
		Task_planner_bin_number_raw,Current_target_item_raw= Receive_Data_Task_Planner()	# get data from Task Planner
		
		Current_bin_number,UR_moving_status = Analyse_Bin_Task_Planner(Task_planner_bin_number_raw)
		if Task_planner_bin_number_raw!="#":
		    
		    if initialization_mode==True:
		        Current_item_count,Current_item_name=Analyse_Item_Task_Planner(Current_bin_number,int(Current_target_item_raw))
		        Update_item_name_list(Current_item_count,Current_item_name)
		        #Current_item_count=2
		        
		        #Current_item_name=["genuine_joe_plastic_stir_sticks","munchkin_white_hot_duck_bath_toy"]
		    elif initialization_mode==False:
		        Current_item_count=2
		        
		        Current_item_name=["genuine_joe_plastic_stir_sticks","munchkin_white_hot_duck_bath_toy"]
		
		if (UR_moving_status== False):
			
			
			if (Current_bin_number != 0):
			    print "******************************************-------Moving module failed! \n"	
			    Error_Recovery(1)
			    

		elif (UR_moving_status== True):
			print "######################################-------Moving module succeed! \n"			
			UR_moving_status=False
			
			
			Capture_Module(Current_item_count,Current_bin_number)


			if (capture_status==False or capture_status_server==False):
				print "***************************************--------Capture module failed!\n"
				Error_Recovery(2)


			elif (capture_status==True and capture_status_server==True):
			    print "######################################--------Capture module succeed! \n"
			    
			    
			    Identification_Module(Current_item_count,Current_item_name,folder_addr_from_capture_module )
			    
			    if (identification_status==False):
				
				    print "**************************************------Identification module failed ! \n"
				    print "I will capture again and retry identification."
				    
				    Error_Recovery(3)
			        
			    if (identification_status==True):       # not using 'elif' because variable will be updated after Error Recovery
			        print "#####################################------Identification module succeed! \n"
			        
			        Registration_Module(blob_index,identification_rotations,folder_addr_from_capture_module,Current_item_name[0])
			        
			        if (registration_status==False):
			            print "**************************************-------Registration module failed! \n"
			            print "I will re-capture, re-identify and retry registration."
			            Error_Recovery(4)
			            
			        if (registration_status==True):     # not using 'elif' because variable will be updated after Error Recovery
			            
			            print "#####################################-------Registration module succeed! \n"
			            new_target_pose=[0,0,0,0,0,0,0,0,0]
			            new_target_pose=pose_calculation(item_pose)
			            #new_target_pose=[0.01,-0.06,0.25,0,0,0, 0.142, 0.095, 0.107]######################
			            out_of_range=True
			            out_of_range=Out_of_range_detector(new_target_pose)
			            
			            if out_of_range == True:
			                print "Registered Posture is Out of Range!!"
			                print "Relative posture: " + str(new_target_pose)
			                print "I will re-capture, re-identify and retry registration."
			                Error_Recovery(5)
			            if out_of_range == False:   # not using 'elif' because variable will be updated after Error Recovery
			                print "Registered posture is feasible!"
			                print "Relative posture: " + str(new_target_pose)
			                Grasping_Module(0,Current_item_name[0],new_target_pose) #get target pose from grasping
			                print "\n-------- Seeking Approval from Task Planner--------"
			                Update_Coeff_Seek_Permission_Task_Planner()
			                if (Approval_Task_Planner == False):
			                    print "Grasping action is not allowed from Task Planner!"
			                    print "UR will go back to init position."
			                    Move_robot_to_init_pose()
			                elif (Approval_Task_Planner == True):
			                    print "Grasping Module is activated! "
			                    print "\n-------- Activation of Grasping module--------"
			                    print "Starting Grasping action now !"
			                    print "Request for target pose from Grasping Module now !"
			                    
			                    #permission=input("Grasp?  (1=yes;2=no).")
			                    permission =1
			                    if permission ==1:
			                        
                                    
			                        if grasping_status==False:
			                            print "#####################################-------Grasping module failed! \n"
			                            print "Fail to get target pose from Grasping Module or collision occurs."
			                            print "I'll try to get target pose again."
			                            Error_Recovery(6)
			                        if grasping_status==True:   # not using 'elif' coz variable will be updated after Err Recovery
			                            print "Successfully getting the grasping pose."
			                            print "Moving robot to grasping target pose now."
			                            activation_grasping_or_suction_routine(1) # turn on, move to grasping target pose, then grasp
			                            
			                            if Safety_Permission == True :
			                              
			                                if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==False) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==False):
			                                    print "#####################################-------Grasping module failed! \n"
			                                    print "Fail to grasp/suck the item in the bin."
			                                    print "I'll get out of the bin and re-capture, re-identify, re-register,and re-grasp/suck again."
			                                    Error_Recovery(7)
			                                if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==True) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==True):   # not using 'elif' because variable will be updated after Error Recovery
			                                    
			                                    print "Successfully grasp/suck the item."
			                                    print "Moving robot out of bin now."
			                                    Move_robot_from_bin_to_order_bin_1st(Current_bin_number,Pose_from_grasping)
			                                    checking_grasping_status()
			                                    
			                                    if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==False) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==False):
			                                        print "#####################################-------Grasping module failed! \n"
			                                        print "Item dropped in the bin."
			                                        print "I'll get out of the bin and re-capture, re-identify, re-register,and re-grasp/suck again."
			                                        Error_Recovery(8)
			                                    if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==True) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==True):   # not using 'elif' because variable will be updated after Error Recovery
			                                        print "Item not dropped in the bin."
			                                        print "Moving robot to init pose now."
			                                        Move_robot_from_bin_to_order_bin_2nd(Current_bin_number)
			                                        checking_grasping_status()
			                                        
			                                        if ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==False) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==False):
			                                            print "#####################################-------Grasping module failed! \n"
			                                            print "Item dropped on the ground."
			                                            print "I'll go to init pose and inform task planner."
			                                            Error_Recovery(9) # non-recoverable error
			                                        elif ((grasping_method==3 or grasping_method==4 or grasping_method==6 or  grasping_method==7 or grasping_method==10 or grasping_method==11) and grasping_status==True) or ((grasping_method==1 or grasping_method==2 or grasping_method==5 or grasping_method==6 or  grasping_method==7 or grasping_method==8) and item_sucked==True):
			                                            print "Item not dropped on ground."
			                                            print "Moving robot to order bin now."
			                                            Move_robot_from_bin_to_order_bin_3rd(Current_bin_number)
			                                            activation_grasping_or_suction_routine(0) # turn off/release
			                                            Move_robot_from_order_bin_to_init(Current_bin_number)
			                                            Task_Status_Task_Planner(2)		#  feedback to task planner that task done
			                            
			                            elif Safety_Permission == False :
			                            	Move_robot_to_init_pose()
			                            	Send_Coefficient_To_Task_Planner(2,2,2)
			                            	if Testing_with_Teck_Hou==True:
			                            		testing=Get_Confirmation_Task_planner()
					                        
			                     
			                    
			                    else:
			                    	Move_robot_to_init_pose()
			                    	Send_Coefficient_To_Task_Planner(2,2,2)
			                    	if Testing_with_Teck_Hou==True:
			                    		testing=Get_Confirmation_Task_planner()
			                    
			                        
			                     
			                     
			                     
			                     
			                     
			                     
			                     
			                     
			                     
			                    
  


























