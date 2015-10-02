#!/usr/bin/env python

import socket
import time
import rospy
import roslib
import sys
import os
import shutil
import random

from std_msgs.msg import String
from identification.srv import *

#######################################################################################
###############-----------Functions Declared-------------##############################
#######################################################################################

def Identification_Service_Call(item_count,item_list,image_addr):
    
    #print "item count : " + str(item_count)
    #print "item list : " + str(item_list)
    #print "image addr : " + str(image_addr)  
    
    rospy.wait_for_service('identify')
    try:
        service_status = rospy.ServiceProxy('identify', SRV_Identification)
        feedback = service_status(image_addr,item_count,item_list)
        return feedback.blob_index, feedback.confidence, feedback.rotations, feedback.versions
        #print "Identification rotations: " + str(feedback.rotations)
    except rospy.ServiceException, e:
        print "Identification Service call failed: %s"%e
        return 0,0
    
#######################################################################################
#####################-----------Main Program-------------##############################
#######################################################################################

if __name__ == '__main__': 

	random.seed()
		

	rawDataDir = os.getcwd() + "/../test_data/test_data"
	
	if (os.path.isdir(rawDataDir)):
		folder_addr_from_capture_module = os.getcwd() + "/../test_data/node_test"
		
		print "Reading test data from " + rawDataDir + "\n"
		
		rgbImages = os.listdir(rawDataDir + "/rgb")
		rgbImages.sort()
		
		successCount = 0
		
		for im_index, im in enumerate(rgbImages):
			shutil.copy2(rawDataDir + "/rgb/" + im, rawDataDir + "/../node_test/rgbImage.png")
			shutil.copy2(rawDataDir + "/mask/" + im, rawDataDir + "/../node_test/maskImage.png")
			shutil.copy2(rawDataDir + "/depth/" + im, rawDataDir + "/../node_test/depthImage.png")

			paramFile = rawDataDir + "/params/" + im
			paramFile = paramFile.replace(".png", ".yml")
			paramFile = paramFile.replace(".jpg", ".yml")

			paramDest = rawDataDir + "/../node_test/param.yml"

			if os.path.isfile(paramFile):
				shutil.copy2(paramFile, paramDest)
			elif os.path.isfile(paramDest):
				os.remove(paramDest)
			
			labelFile = rawDataDir + "/labels/" + im
			labelFile = labelFile.replace(".png", ".txt")
			labelFile = labelFile.replace(".jpg", ".txt")
			
			# print "Reading labels from " + labelFile
			text_file = open(labelFile, "r")
			lines = text_file.readlines()
			text_file.close()
			
			uniqueIdentities = []
			
			for index, L in enumerate(lines):
				lines[index] = L.replace("\r","")
				lines[index] = lines[index].replace("\n","")
				if (lines[index].rfind(" ") != -1):
					lines[index] = lines[index][:lines[index].rfind(" ")]
				isUnique = 1
				for q in uniqueIdentities:
					if (q == lines[index]):
						isUnique = 0
				if isUnique == 1:
					uniqueIdentities.append(lines[index])
			
			Current_item_count = len(lines)
			Current_item_name = lines
			
			###############################################
			#####---Calling Identification Module-----#####
			###############################################
			random_index=random.randint(0, Current_item_count-1)
			
			print("Test " + "{0:03d}".format(im_index+1) + ": Searching (" + str(len(uniqueIdentities)) + ") unique classes for item <" + Current_item_name[random_index] + ">")
			
			temp_name = Current_item_name[0]
			Current_item_name[0] = Current_item_name[random_index]
			Current_item_name[random_index] = temp_name
			
			blob_index = 0
			identification_confidence=0
			identification_rotations=[]
			identification_versions=[]
			
			for i in  range(0,45):
				identification_rotations.append(0)
			
			blob_index,identification_confidence,identification_rotations,identification_versions=Identification_Service_Call(Current_item_count,Current_item_name,folder_addr_from_capture_module)

			if (lines[random_index] == lines[blob_index]):
				print "\t  Success. Confidence = " + "{0:.2f}".format(identification_confidence) + "\n"
				successCount = successCount + 1
			else:
				print "\t  FAILURE. Confidence = " + "{0:.2f}".format(identification_confidence) + "\n"
				time.sleep(3.0)
			
		print "Succeeded in " + str(successCount) + "/" + str(len(rgbImages)) + " tests.\n"
		
	else:
		print "ERROR! Couldn't find test data in " + rawDataDir

	
