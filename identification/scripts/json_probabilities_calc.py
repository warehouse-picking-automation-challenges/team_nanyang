#!/usr/bin/env python

import os
import json
import yaml
import csv

from pprint import pprint

#######################################################################################
###############-----------Functions Declared-------------##############################
#######################################################################################

def Read_IPPM_CSV(file_address):
	print "Reading csv file: <" + file_address[file_address.rfind("/")+1:] + ">"
	ippm = []
	with open(file_address, 'rb') as csvfile:
		ippm_reader = csv.reader(csvfile, delimiter=',')
		for i,row in enumerate(ippm_reader):
			ippm.append([])
			for j,el in enumerate(row):
				ippm[i].append(el)	
	return ippm
	
def Read_JSON(file_address):
	print "Reading json file <" + file_address[file_address.rfind("/")+1:] + ">"
	json_data=open(file_address)
	data = yaml.load(json_data)
	json_data.close()
	# pprint(data)
	
	## Read in bin names
	bin_list = []
	for index, B in enumerate(data["bin_contents"]):
		bin_list.append(B)
	bin_list.sort()
	# pprint(bin_list)
	
	## Read in item names
	bin_contents = []
	for B in bin_list:
		#pprint(data["bin_contents"][B])
		bin_contents.append(data["bin_contents"][B])

	
	## Read in target items
	work_order = []
	for index, W in enumerate(data["work_order"]):
		for B in bin_list:
			if (W["bin"] == B):
				work_order.append(W["item"])
	
	return bin_contents,work_order
	
def Get_ALL_Items(directory_address):
	print "Reading directory <" + directory_address[directory_address.rfind("/")+1:] + ">"
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
		idx = full_items_list.index(work_order[i])
		# print("target " + str(i) + " item is " + work_order[i] + " at " + str(idx))
		
		for j, J in enumerate(bin_contents[i]):
			idx2 = full_items_list.index(bin_contents[i][j])
			# print("bin " + str(i) + " item " + str(j) + " = " + bin_contents[i][j] + " at " + str(idx2))
			prob = prob * float(ippm[idx][idx2])
		
		bin_id_priors.append(prob)
	return bin_id_priors
   
#######################################################################################
#####################-----------Main Program-------------##############################
#######################################################################################

if __name__ == '__main__': 
	
	ippm_file = os.path.dirname(os.path.realpath(__file__)) + "/../params/identification_pp_matrix.csv"
	print(ippm_file)
	ippm = Read_IPPM_CSV(ippm_file) # ippm[i][j]
			
	json_file = os.path.dirname(os.path.realpath(__file__)) + "/../params/example.json"
	bin_contents,work_order = Read_JSON(json_file)
	
	yml_library = os.path.dirname(os.path.realpath(__file__)) + "/../yml_library/yml_library"
	full_items_list = Get_ALL_Items(yml_library)
	
	bin_id_priors = Calculate_bin_identification_priors(ippm, bin_contents, work_order, full_items_list)
	pprint(bin_id_priors)
