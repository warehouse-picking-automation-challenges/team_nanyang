#!/usr/bin/env python

import rospy
import math
import time
import decimal
import numpy

from item_library_new import *

def coordinate_quaternion( pos_from_manager, dimension, item ):
	reply_pos = [0] *15*8
	
	### assign dimension ###
	w_x = dimension[0]
	h_y = dimension[1]
	l_z = dimension[2]
	grasp_method_1 = dimension[3] 	#front suction only
	grasp_method_2 = dimension[4] 	#bottom suction only
	grasp_method_3 = dimension[5]		#big grasp only
	grasp_method_4 = dimension[6]		#small grasp only
	grasp_method_5 = dimension[7]		#bottom suction from side only
	grasp_method_6 = dimension[8]		#front suction + big grasp
	grasp_method_7 = dimension[9]		#bottom suction + big grasp
	grasp_method_8 = dimension[10]	#bottom suction thin object only
	grasp_method_9 = dimension[11]	#empty
	grasp_method_10 = dimension[12]	#Use Grasp to straighten Item 
	grasp_method_11 = dimension[13]	#Use Fingers to push down Item
	grasp_method_12 = dimension[14]	#Use Fingers to push down Item
	grasp_method_13 = dimension[15]	#Use Fingers to push down Item
	grasp_method_14 = dimension[16]	#Use Fingers to push down Item
	grasp_method_15 = dimension[17]	#Use Fingers to push down Item
	box = dimension[21]
	weight = dimension[22]

	bin_type = dimension[40]
	bin_size_x = dimension[41]
	bin_size_y = dimension[42]
	
	limit_y_up_suctionlift = dimension[43]
	limit_y_up_suctionlower = dimension[44]
	limit_y_up_suctionside = dimension[45]

	limit_y_do_suction = dimension[46]
	limit_y_do_grapmax = dimension[47]
	limit_y_do_grapmin = dimension[48]
	limit_y_do_suctionside = dimension[49]

	limit_x_neg_neutral = dimension[50]
	limit_x_neg_max = dimension[51]
	limit_x_neg_min = dimension[52]
	limit_x_neg_side = dimension[53]

	limit_x_pos_neutral = dimension[54]
	limit_x_pos_max = dimension[55]
	limit_x_pos_min = dimension[56]
	limit_x_pos_side = dimension[57]

	limit_x_pos_angle = dimension[58]
	limit_x_neg_angle = dimension[59]
	limit_y_pos_angle = dimension[60]
	limit_y_neg_angle = dimension[61]

	limit_z = dimension[62]

	confi_method_1 = 0
	confi_method_2 = 0
	confi_method_3 = 0
	confi_method_4 = 0
	confi_method_5 = 0
	confi_method_6 = 0
	confi_method_7 = 0
	confi_method_8 = 0
	confi_method_9 = 0
	confi_method_10 = 0
	confi_method_11 = 0
	confi_method_12 = 0
	confi_method_13 = 0
	confi_method_14 = 0
	confi_method_15 = 0

	### assign & determine coordinate ###
	xx = round(pos_from_manager[0], 3)
	yy = round(pos_from_manager[1], 3)
	zz = round(pos_from_manager[2], 3)
	rot_x = round(math.degrees(pos_from_manager[3]), 3)
	rot_y = round(math.degrees(pos_from_manager[4]), 3)
	rot_z = round(math.degrees(pos_from_manager[5]), 3)
	w = round(pos_from_manager[6], 3)
	h = round(pos_from_manager[7], 3)
	d = round(pos_from_manager[8], 3)
	bin_num = pos_from_manager[9]

	txx = xx
	tyy = yy
	tzz = zz
	trot_x = rot_x
	trot_y = rot_y
	trot_z = rot_z

	if box == 1:
		classf = "Suckable & Graspable"
	elif box == 2:
		classf = "Graspable Only"
	elif box == 3:
		classf = "Specially Difficult Item"

	## To calculate rotation-y & rotation-x and set limits ##
	if trot_y >= limit_y_neg_angle and trot_y <= limit_y_pos_angle:	
		cal_rot_y = trot_y
	else: 
		if trot_y <= limit_y_neg_angle:
			cal_rot_y = limit_y_neg_angle
		elif trot_y >= limit_y_pos_angle:
			cal_rot_y = limit_y_pos_angle
		print "+---------> rotation y - out of range (Corrected)"

	if trot_x >= limit_x_neg_angle and trot_x <= limit_x_pos_angle:	
		cal_rot_x = trot_x
	else: 
		if trot_x <= limit_x_neg_angle:
			cal_rot_x = limit_x_neg_angle
		elif trot_x >= limit_x_pos_angle:
			cal_rot_x = limit_x_pos_angle	
		print "+---------> rotation x - out of range (Corrected)"
	Z_offset_registration = 0.170
	Z_limit = 0.200 + Z_offset_registration
	Z_limit_bot = 0 + Z_offset_registration
	if bin_num == 1 or bin_num == 3 or bin_num == 10 or bin_num == 12:
		bin_limit_x = 0.250
		bin_limit_y = 0.225
		if bin_num == 1 or bin_num == 3:
			Z_limit = 0.160 + Z_offset_registration
			bin_limit_y = 0.188
	elif bin_num == 2 or bin_num == 11:
		bin_limit_x = 0.300
		bin_limit_y = 0.225
	elif bin_num == 4 or bin_num == 6 or bin_num == 7 or bin_num == 9:
		bin_limit_x = 0.250
		bin_limit_y = 0.188
	elif bin_num == 5 or bin_num == 8:
		bin_limit_x = 0.300
		bin_limit_y = 0.188
		Z_limit = 0.250 + Z_offset_registration
	
	h_left = bin_limit_y - h
	w_left = bin_limit_x - w

	print "##Pos from Manager##"
	print "Point (x, y, z):		" + str(xx) + " , " + str(yy) + " , " + str(zz)
	print "Rotation (Rx, Ry, Rz):		" + str(rot_x) + " , " + str(rot_y) + " , " + str(rot_z)
	print "Width, Height, Depth:		" + str(w) + " , " + str(h) + " , " + str(d)
	print "Item Dimension (Origin):	" + str(w_x) + " , " + str(h_y) + " , " + str(l_z) 
	print "Bin Number: " + str(bin_num)
	print "Bin Height: " + str(bin_limit_y) + "  & Width: " + str(bin_limit_x)
	print "Weight: " + str(weight) + " grams"
	
	## Grasp Planning ##
	EE_move_up_distance = -0.020
	EE_side_suction_move_up = 0.035
	EE_move_forward_distance_front_suction = 0.025
	safety_buffer = 0.007
	safety_buffer_side = 0.005

	sort = 0
	bin_base_offset = 0.014 - 0.020 #offset for registration
	suction_cup_radius = 0.010
	suction_cup_diameter = suction_cup_radius *2
	finger_radius = 0.006
	finger_from_front_suc_y = 0.060
	finger_from_front_suc_z = 0.050
	bot_suc_from_front_suc_y = 0.040
	bot_suc_from_front_suc_z = 0.050

	grasp_strategy = 0
	grasping_coeff = 0
	call_error = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_1 > 0: #Front Suction
		print "+---------> Thinking method 1:"
		if d < 0.120 and w > suction_cup_diameter:

			bounding_box_left = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_right = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_up = 0.075 + safety_buffer + (-1*EE_move_up_distance)
			bounding_box_down = 0.045 + safety_buffer

			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right
			
			if tyy <= -1*tyy_limit_low and tyy >= -1*tyy_limit_high: 	
				tyy_cal = tyy
			else:
				if tyy > -1*tyy_limit_low and h >= tyy_limit_low + suction_cup_radius:
					tyy_cal = -1*tyy_limit_low
				elif tyy < -1*tyy_limit_high:
					tyy_cal = -1*tyy_limit_high
				else:
					call_error = 1
					print "+---------> method 1 failed: Y out of range"

			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 1 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 1 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 1 failed: X out of range"

			if tzz - d/2 <= Z_limit and tzz - d/2 > Z_limit_bot:
				tzz_cal = tzz - d/2 
			else:
				call_error = 1
				print "+---------> method 1 failed: Z out of range"

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal - EE_move_forward_distance_front_suction
			rot_x = cal_rot_x
			rot_y = cal_rot_y
			rot_z = 0
			grasp_strategy = 1
			grasping_coeff = 0.86 *grasp_method_1
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 1 failed: Item dimension not suitable"

	grasp_strategy = 0
	grasping_coeff = 0
	call_error = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_2 > 0 or grasp_method_8 > 0: #Bottom Suction
		if h_left > 0.09 and w > suction_cup_diameter and d > suction_cup_diameter:

			bounding_box_left = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_right = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_up = 0.05 + safety_buffer
			bounding_box_down = 0.045 + safety_buffer

			tyy_2 = tyy - bot_suc_from_front_suc_y
			tzz_2 = tzz + bot_suc_from_front_suc_z
			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right
			
			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 2 or 8 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 2 or 8 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 2 or 8 failed: X out of range"

			if tzz_2 <= Z_limit and tzz_2 > Z_limit_bot + 0.005:
				tzz_cal = tzz_2 
			else:
				call_error = 1
				print "+---------> method 2 or 8 failed: X out of range"

			if h < 0.05:
				xx = txx_cal
				yy = -0.090 #Special value
				zz = tzz_cal
				rot_x = 0
				rot_y = 0
				rot_z = 0
				grasp_strategy = 8
				grasping_coeff = 0.98 *grasp_method_8
			else:
				xx = txx_cal
				yy = -1*tyy_limit_high
				zz = tzz_cal
				rot_x = 0
				rot_y = 0
				rot_z = 0
				grasp_strategy = 2
				grasping_coeff = 0.99 *grasp_method_2

			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 2 & 8 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_3 > 0: #Big Grasp
		if w < 0.145 and w > 0.085:

			bounding_box_left = 0.09 + safety_buffer + safety_buffer_side
			bounding_box_right = 0.09 + safety_buffer + safety_buffer_side 
			bounding_box_up = 0.07 + safety_buffer + (-1*EE_move_up_distance)
			bounding_box_down = 0.06 + safety_buffer

			tyy_3 = tyy - finger_from_front_suc_y
			tzz_3 = tzz + finger_from_front_suc_z

			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right
			
			if tyy_3 <= -1*tyy_limit_low and tyy_3 >= -1*tyy_limit_high: 	
				tyy_cal = tyy_3
			else:
				if tyy_3 >= -1*tyy_limit_low and h >= tyy_limit_low + finger_radius:
					tyy_cal = -1*tyy_limit_low - finger_from_front_suc_y
				elif tyy_3 < -1*tyy_limit_high:
					tyy_cal = -1*tyy_limit_high
				else:
					call_error = 1
					print "+---------> method 3 failed: Y out of range"

			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 3 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 3 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 3 failed: X out of range"

			if tzz_3 <= Z_limit and tzz_3 > Z_limit_bot:
				tzz_cal = tzz_3 
			else:
				call_error = 1
				print "+---------> method 3 failed: Z out of range" + str(tzz_3) + str(Z_limit) + str(Z_limit_bot)

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = cal_rot_y
			rot_z = 0
			grasp_strategy = 3
			grasping_coeff = 0.8 *grasp_method_3
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 3 failed: Item dimension not suitable"
	
	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_4 > 0: #Small Grasp
		if w <= 0.085:

			bounding_box_left = 0.045 + safety_buffer + safety_buffer_side 
			bounding_box_right = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_up = 0.070 + safety_buffer + (-1*EE_move_up_distance)
			bounding_box_down = 0.060 + safety_buffer

			tyy_4 = tyy - finger_from_front_suc_y
			tzz_4 = tzz + finger_from_front_suc_z
			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right
			
			if tyy_4 <= -1*tyy_limit_low and tyy_4 >= -1*tyy_limit_high: 	
				tyy_cal = tyy_4
			else:
				if tyy_4 > -1*tyy_limit_low and h >= tyy_limit_low + finger_radius:
					tyy_cal = -1*tyy_limit_low - finger_from_front_suc_y
				elif tyy_4 < -1*tyy_limit_high:
					tyy_cal = -1*tyy_limit_high
				else:
					call_error = 1
					print "+---------> method 4 failed: X out of range"

			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 4 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 4 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 4 failed: X out of range"

			if tzz_4 <= Z_limit and tzz_4 > Z_limit_bot:
				tzz_cal = tzz_4 
			else:
				call_error = 1
				print "+---------> method 4 failed: Z out of range"

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = cal_rot_y
			rot_z = 0
			grasp_strategy = 4
			grasping_coeff = 0.7 * grasp_method_4
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 4 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_5 > 0: #Side Suction
		if h > 0.06 and d > suction_cup_diameter:

			bounding_box_up = 0.05 + safety_buffer
			bounding_box_down = 0.045 + safety_buffer 
			if  txx >= 0:
				bounding_box_left = 0.045 + safety_buffer + EE_side_suction_move_up + safety_buffer_side 
				bounding_box_right = 0.045 + safety_buffer + safety_buffer_side 
			else:
				bounding_box_left = 0.045 + safety_buffer + safety_buffer_side 
				bounding_box_right = 0.045 + safety_buffer + EE_side_suction_move_up + safety_buffer_side 

			tzz_5 = tzz + bot_suc_from_front_suc_z
			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right

			if tyy <= -1*tyy_limit_low and tyy >= -1*tyy_limit_high: 	
				tyy_cal = tyy
			else:
				if tyy > -1*tyy_limit_low and h >= tyy_limit_low + suction_cup_radius:
					tyy_cal = -1*tyy_limit_low
				elif tyy < -1*tyy_limit_high:
					tyy_cal = -1*tyy_limit_high
				else:
					call_error = 1
					print "+---------> method 5 failed: Y out of range"

			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 5 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 5 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 5 failed: X out of range"

			if tzz_5 <= Z_limit and tzz_5 > Z_limit_bot:
				tzz_cal = tzz_5 
			else:
				call_error = 1
				print "+---------> method 5 failed: Z out of range"

			if txx < 0:
				rot_z = 90
			else:
				rot_z = -90

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = rot_z
			grasp_strategy = 5
			grasping_coeff = 0.6 * grasp_method_5

			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 5 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_6 > 0: #Front Suction + Small Grasp
		if w > suction_cup_diameter and w < 0.085:

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = rot_z
			grasp_strategy = 6
			grasping_coeff = 0.3 * grasp_method_6
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 6 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_7 > 0: #Front Suction + Big Grasp
		if w > suction_cup_diameter and w < 0.145:

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = rot_z
			grasp_strategy = 7
			grasping_coeff = 0.35 * grasp_method_7
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 7 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_9 > 0: #Special soft-toy
		if w < 0.085:

			bounding_box_left = 0.045 + safety_buffer + safety_buffer_side 
			bounding_box_right = 0.045 + safety_buffer + safety_buffer_side
			bounding_box_up = 0.07 + safety_buffer + (-1*EE_move_up_distance)
			bounding_box_down = 0.06 + safety_buffer

			tyy_9 = tyy - finger_from_front_suc_y
			tzz_9 = tzz + finger_from_front_suc_z
			tyy_limit_low = bounding_box_down + bin_base_offset
			tyy_limit_high = bin_limit_y - bounding_box_up + bin_base_offset
			txx_limit_left = bin_limit_x/-2 + bounding_box_left
			txx_limit_right = bin_limit_x/2 - bounding_box_right
			
			if tyy_9 <= -1*tyy_limit_low and tyy_9 >= -1*tyy_limit_high: 	
				tyy_cal = tyy_9
			else:
				if tyy_9 > -1*tyy_limit_low and h >= tyy_limit_low + finger_radius:
					tyy_cal = -1*tyy_limit_low - finger_from_front_suc_y
				elif tyy_9 < -1*tyy_limit_high:
					tyy_cal = -1*tyy_limit_high - finger_from_front_suc_y
				else:
					call_error = 1
					print "+---------> method 4 failed: X out of range"

			if txx >= txx_limit_left and txx <= txx_limit_right: 	
				txx_cal = txx
			else:
				if txx < txx_limit_left: 
					item_width_limit = txx + w/2 - suction_cup_radius
					if item_width_limit >= txx_limit_left:
						txx_cal = txx_limit_left
					else:
						call_error = 1
						print "+---------> method 9 failed: X out of range"
				elif txx > txx_limit_right:
					item_width_limit = txx - w/2 + suction_cup_radius
					if item_width_limit <= txx_limit_right:
						txx_cal = txx_limit_right
					else:
						call_error = 1
						print "+---------> method 9 failed: X out of range"
				else:
					call_error = 1
					print "+---------> method 9 failed: X out of range"

			if tzz_9 <= Z_limit and tzz_9 > Z_limit_bot:
				tzz_cal = tzz_9 
			else:
				call_error = 1
				print "+---------> method 9 failed: Z out of range"

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = cal_rot_y
			rot_z = 0
			grasp_strategy = 9
			grasping_coeff = 0.5 * grasp_method_9
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 9 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_10 > 0: #Use Grasp to straighten Item 
		if w < 0.145:

			xx = txx_cal
			yy = tyy_cal
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = 0
			grasp_strategy = 10
			grasping_coeff = 0.11 * grasp_method_10
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 10 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_11 > 0: #Use Fingers to push down Item 
		if h > 0.06 and d < 0.04:

			xx = txx_cal
			yy = h*-1 - 0.020
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = 0
			grasp_strategy = 11
			grasping_coeff = 0.11 * grasp_method_11
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 11 failed: Item dimension not suitable"

	call_error = 0
	grasp_strategy = 0
	grasping_coeff = 0
	txx_cal = txx
	tyy_cal = tyy
	tzz_cal = tzz
	if grasp_method_12 > 0: #Use Fingers to push down Item 
		if h > 0.06 and w < 0.04:

			xx = txx_cal
			yy = h*-1 - 0.020
			zz = tzz_cal
			rot_x = 0
			rot_y = 0
			rot_z = 0
			grasp_strategy = 12
			grasping_coeff = 0.11 * grasp_method_12
			if call_error != 1:
				reply_pos[sort] = grasping_coeff
				reply_pos[sort + 1] = xx
				reply_pos[sort + 2] = yy
				reply_pos[sort + 3] = zz
				reply_pos[sort + 4] = math.radians(rot_x)
				reply_pos[sort + 5] = math.radians(rot_y)
				reply_pos[sort + 6] = math.radians(rot_z)
				reply_pos[sort + 7] = grasp_strategy
				sort = sort + 8
		else:
			print "+---------> method 12 failed: Item dimension not suitable"

	print "reply from BRAIN: Item is " + name 
	print 'Object is a ' + classf

	if grasp_strategy == 0:
		print "unable to compute grasp strategy."
	else:	
		tsort = sort/8
		print "Total possible strategies sent to Manager = " + str(tsort)

	i = 0
	while i < 120:
		if reply_pos[i] != 0:
			print reply_pos[i], reply_pos[i+1], reply_pos[i+2], reply_pos[i+3], reply_pos[i+4], reply_pos[i+5], reply_pos[i+6], reply_pos[i+7] 
		i += 8
	print "\n"
	return reply_pos

if __name__ == "__main__":
	print "I am thinking....HARD"
