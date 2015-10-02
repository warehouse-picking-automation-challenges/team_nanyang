#!/usr/bin/env python

import rospy
import math
import time
import decimal

item = 0
name = ""

#[0] = "paper_mate_12_count_mirado_black_warrior"		0	159
#[1] = "elmers_washable_no_run_school_glue"				1	160
#[2] = "laugh_out_loud_joke_book"							0	161
#[3] = "feline_greenies_dental_treats"						1	162 
#[4] = "kong_sitting_frog_dog_toy"							0	163
#[5] = "genuine_joe_plastic_stir_sticks"					0	164 
#[6] = "expo_dry_erase_board_eraser"						1	165
#[7] = "rolodex_jumbo_pencil_cup"							0	166
#[8] = "kong_duck_dog_toy"										0	167
#[9] = "safety_works_safety_glasses"						0	168	
#[10] = "mark_twain_huckleberry_finn"						0	169 
#[11] = "first_years_take_and_toss_straw_cup"			0	170
#[12] = "champion_copper_plus_spark_plug"					0	171
#[13] = "cheezit_big_original"								1	172 
#[14] = "munchkin_white_hot_duck_bath_toy"				0	173 
#[15] = "kyjen_squeain_eggs_plush_puppies"				0	174
#[16] = "sharpie_accent_tank_style_highlighters"		1	175
#[17] = "stanley_66_052"										0	176
#[18] = "kong_air_dog_squeakair_tennis_ball"				0	177
#[19] = "oreo_meg_stuff"										1	178 
#[20] = "crayola_64_ct"											0	179
#[21] = "mummt_helper_outlet_plugs"							0	180
#[22] = "highland_6539_self_stick_notes"					0	181
#[23] = "mead_index cards"										0	182
#[24] = "dr_browns_bottle_brush"								0	183

def target_dimension( selection, bin_number ):
	target = [0] *63
	item = selection - 159
	name = ""
	bin_num = bin_number

	if item == 0: 
		name = "paper_mate_12_count_mirado_black_warrior"
		target[0] = 0.048 #xx
		target[1] = 0.195 #xy 
		target[2] = 0.016 #xz 		
		
		#possible grasping methods		
		target[3] = 0.8 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.3 # 3. big grasp only
		target[6] = 0.3 # 4. small grasp only 
		target[7] = 0.5 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. 

		#Item Manipulation
		target[12] = 0.0 #Use Grasp to straighten Item 
		target[13] = 0.5 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 75.0 #Weight

	elif item == 1: 
		name = "elmers_washable_no_run_school_glue"
		target[0] = 0.065 #xx
		target[1] = 0.148 #xy 
		target[2] = 0.030 #xz 	
		
		#possible grasping methods		
		target[3] = 0.7 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.3 # 3. big grasp only
		target[6] = 0.3 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.5 #Use Grasp to straighten Item 
		target[13] = 1.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 266.0 #Weight

	elif item == 2: 
		name = "laugh_out_loud_joke_book"
		target[0] = 0.108 #xx
		target[1] = 0.009 #xy 
		target[2] = 0.178 #xz 			
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.0 # 3. big grasp only
		target[6] = 0.0 # 4. small grasp only 
		target[7] = 0.8 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.0 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 81.8 #Weight

	elif item == 3: 
		name = "feline_greenies_dental_treats"
		target[0] = 0.167 #xx
		target[1] = 0.216 #xy 
		target[2] = 0.045 #xz 		
		
		#possible grasping methods		
		target[3] = 0.2 # 1. front suction only
		target[4] = 0.7 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.3 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.4 # 6. front suction + small grasp
		target[9] = 0.5 # 7. front suction + big grasp 
		target[10] = 0.7 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.2 #Use Grasp to straighten Item 
		target[13] = 0.5 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 168.0 #Weight

	elif item == 4: 
		name = "kong_sitting_frog_dog_toy"
		target[0] = 0 #xx
		target[1] = 0 #xy 
		target[2] = 0 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.4 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.0 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 31.0 #Weight

	elif item == 5: 
		name = "genuine_joe_plastic_stir_sticks"
		target[0] = 0.145 #xx
		target[1] = 0.095 #xy 
		target[2] = 0.105 #xz 		
		
		#possible grasping methods		
		target[3] = 0.9 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.3 # 3. big grasp only
		target[6] = 0.2 # 4. small grasp only 
		target[7] = 0.8 # 5. Side suction
		target[8] = 0.1 # 6. front suction + small grasp
		target[9] = 0.4 # 7. front suction + big grasp 
		target[10] = 0.8 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.2 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 275 #Weight

	elif item == 6: 
		name = "expo_dry_erase_board_eraser"
		target[0] = 0.135 #xx
		target[1] = 0.055 #xy 
		target[2] = 0.038 #xz 		
		
		#possible grasping methods		
		target[3] = 0.8 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.8 # 5. Side suction
		target[8] = 0.3 # 6. front suction + small grasp
		target[9] = 0.3 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.2 #Use Grasp to straighten Item 
		target[13] = 0.2 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 21 #Weight

	elif item == 7: 
		name = "rolodex_jumbo_pencil_cup"
		target[0] = 0.110 #xx
		target[1] = 0.135 #xy 
		target[2] = 0.038 #xz 		
		
		#possible grasping methods		
		target[3] = 0.3 # 1. front suction only
		target[4] = 0.6 # 2. bottom suction only
		target[5] = 0.7 # 3. big grasp only
		target[6] = 0.7 # 4. small grasp only 
		target[7] = 0.8 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.4 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 89.4 #Weight

	elif item == 8: 
		name = "kong_duck_dog_toy"
		target[0] = 0 #xx
		target[1] = 0 #xy 
		target[2] = 0 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.5 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 31.31 #Weight

	elif item == 9: 
		name = "safety_works_safety_glasses"
		target[0] = 0.162 #xx
		target[1] = 0.065 #xy 
		target[2] = 0.040 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.5 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 19.0 #Weight

	elif item == 10: 
		name = "mark_twain_huckleberry_finn"
		target[0] = 0.129 #xx
		target[1] = 0.228 #xy 
		target[2] = 0.015 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.0 # 3. big grasp only
		target[6] = 0.0 # 4. small grasp only 
		target[7] = 0.8 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.0 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 169 #Weight

	elif item == 11: 
		name = "first_years_take_and_toss_straw_cup"
		target[0] = 0.090 #xx
		target[1] = 0.228 #xy 
		target[2] = 0.1 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.4 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.2 #Use Grasp to straighten Item 
		target[13] = 0.2 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 129 #Weight in grams

	elif item == 12: 
		name = "champion_copper_plus_spark_plug"
		target[0] = 0.098 #xx
		target[1] = 0.022 #xy 
		target[2] = 0.026 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.4 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3	
		target[22] = 76 #Weight

	elif item == 13: 
		name = "cheezit_big_original"
		target[0] = 0.190 #xx
		target[1] = 0.230 #xy 
		target[2] = 0.065 #xz 		
		
		#possible grasping methods		
		target[3] = 0.7 # 1. front suction only
		target[4] = 1.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.4 # 6. front suction + small grasp
		target[9] = 0.4 # 7. front suction + big grasp 
		target[10] = 1.0 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.2 #Use Grasp to straighten Item 
		target[13] = 0.2 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 389 #Weight

 	elif item == 14: 
		name = "munchkin_white_hot_duck_bath_toy"
		target[0] = 0.092 #xx
		target[1] = 0.134 #xy 
		target[2] = 0.070 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.5 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 2
		target[22] = 70.6 #Weight
 
	elif item == 15: 
		name = "kyjen_squeakin_eggs_plush_puppies"
		target[0] = 0 #xx
		target[1] = 0 #xy 
		target[2] = 0 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.5 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.0 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 46 #Weight

	elif item == 16: 
		name = "sharpie_accent_tank_style_highlighters"
		target[0] = 0.120 #xx
		target[1] = 0.020 #xy 
		target[2] = 0.135 #xz 		
		
		#possible grasping methods		
		target[3] = 0.4 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 116.2 #Weight

	elif item == 17: 
		name = "stanley_66_052"
		target[0] = 0.100 #xx
		target[1] = 0.020 #xy 
		target[2] = 0.165 #xz 		
		
		#possible grasping methods		
		target[3] = 0.4 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 82.0 #Weight

	elif item == 18: 
		name = "kong_air_dog_squeakair_tennis_ball"
		target[0] = 0.107 #xx
		target[1] = 0.190 #xy 
		target[2] = 0.068 #xz 		
		
		#possible grasping methods		
		target[3] = 0.0 # 1. front suction only
		target[4] = 0.0 # 2. bottom suction only
		target[5] = 0.5 # 3. big grasp only
		target[6] = 0.5 # 4. small grasp only 
		target[7] = 0.0 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.0 # 8. bottom suction for thin item
		target[11] = 0.5 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.0 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 3
		target[22] = 99.0 #Weight

	elif item == 19: 
		name = "oreo_mega_stuf"
		target[0] = 0.215 #xx
		target[1] = 0.185 #xy 
		target[2] = 0.055 #xz 		
		
		#possible grasping methods		
		target[3] = 0.4 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 383 #Weight

	elif item == 20: 
		name = "crayola_64_ct"
		target[0] = 0.144 #xx
		target[1] = 0.126 #xy 
		target[2] = 0.039 #xz 		
		
		#possible grasping methods		
		target[3] = 0.7 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.5 # 6. front suction + small grasp
		target[9] = 0.5 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.2 #Use Grasp to straighten Item 
		target[13] = 0.2 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 375 #Weight

	elif item == 21: 
		name = "mommys_helper_outlet_plugs"
		target[0] = 0.118 #xx
		target[1] = 0.052 #xy 
		target[2] = 0.040 #xz 		
		
		#possible grasping methods		
		target[3] = 0.3 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.3 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.3 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 76 #Weight

	elif item == 22: 
		name = "highland_6539_self_stick_notes"
		target[0] = 0.118 #xx
		target[1] = 0.052 #xy 
		target[2] = 0.040 #xz 		
		
		#possible grasping methods		
		target[3] = 0.3 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.3 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 178 #Weight

	elif item == 23: 
		name = "mead_index_cards"
		target[0] = 0.130 #xx
		target[1] = 0.023 #xy 
		target[2] = 0.080 #xz 		
		
		#possible grasping methods		
		target[3] = 0.4 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 148 #Weight

	elif item == 24: 
		name = "dr_browns_bottle_brush"
		target[0] = 0.108 #xx
		target[1] = 0.313 #xy 
		target[2] = 0.053 #xz 		
		
		#possible grasping methods		
		target[3] = 0.4 # 1. front suction only
		target[4] = 0.9 # 2. bottom suction only
		target[5] = 0.4 # 3. big grasp only
		target[6] = 0.4 # 4. small grasp only 
		target[7] = 0.7 # 5. Side suction
		target[8] = 0.0 # 6. front suction + small grasp
		target[9] = 0.0 # 7. front suction + big grasp 
		target[10] = 0.9 # 8. bottom suction for thin item
		target[11] = 0.0 # 9. Special method to pick small object

		#Item Manipulation
		target[12] = 0.1 #Use Grasp to straighten Item 
		target[13] = 0.1 #Use Fingers to push down Item
		target[14] = 0.0 #Use
		target[15] = 0.0 #Use
		target[16] = 0.0 #Use
		target[17] = 0.0 #Use
		target[18] = 0.0 #Use
		target[19] = 0.0 #Use

		#0-dummy, 1-Suckable & Graspable, 2-Graspable, 3-Special
		target[21] = 1
		target[22] = 73 #Weight

	else:
		print 'item_library: item not found!\n'

	safety_buffer = 0.005
	if bin_num==1 or bin_num==3 or bin_num==10 or bin_num==12:
 		target[40] = 1	#bin_type
		target[41] = 0.250 - safety_buffer	#bin_size_x 
		target[42] = 0.225 - safety_buffer	#bin_size_y 
	
		target[43] = -0.105 + safety_buffer	#limit_y_up_suctionlift 
		target[44] = -0.250 - 0.015 + safety_buffer	#limit_y_up_suctionlower 
		target[45] = -0.105 + safety_buffer	#limit_y_up_suctionside

		target[46] = -0.0 + safety_buffer	#limit_y_do_suction
		target[47] = -0.0 + safety_buffer	#limit_y_do_grapmax 
		target[48] = -0.0 + safety_buffer	#limit_y_do_grapmin
		target[49] = -0.0 + safety_buffer	#limit_y_do_suctionside

		target[50] = -0.0 + safety_buffer	#limit_x_neg_neutral
		target[51] = -0.0 + safety_buffer	#limit_x_neg_max
		target[52] = -0.0 + safety_buffer	#limit_x_neg_min
		target[53] = -0.0 + safety_buffer	#limit_x_neg_side

		target[54] = 0.0 - safety_buffer	#limit_x_pos_neutral
		target[55] = 0.0 - safety_buffer	#limit_x_pos_max
		target[56] = 0.0 - safety_buffer	#limit_x_pos_min
		target[57] = 0.0 - safety_buffer	#limit_x_pos_side

		target[58] = 5.000 #limit_x_pos_angle
		target[59] = 0.000 #limit_x_neg_angle
		target[60] = 10.000 #limit_y_pos_angle
		target[61] = -10.000 #limit_y_neg_angle

		target[62] = 10.000 #limit_z

	elif bin_num==2 or bin_num==11:
 		target[40] = 1	#bin_type
		target[41] = 0.250 - safety_buffer	#bin_size_x 
		target[42] = 0.225 - safety_buffer	#bin_size_y 
	
		target[43] = -0.105 + safety_buffer	#limit_y_up_suctionlift 
		target[44] = -0.250 - 0.015 + safety_buffer	#limit_y_up_suctionlower 
		target[45] = -0.105 + safety_buffer	#limit_y_up_suctionside

		target[46] = -0.0 + safety_buffer	#limit_y_do_suction
		target[47] = -0.0 + safety_buffer	#limit_y_do_grapmax 
		target[48] = -0.0 + safety_buffer	#limit_y_do_grapmin
		target[49] = -0.0 + safety_buffer	#limit_y_do_suctionside

		target[50] = -0.0 + safety_buffer	#limit_x_neg_neutral
		target[51] = -0.0 + safety_buffer	#limit_x_neg_max
		target[52] = -0.0 + safety_buffer	#limit_x_neg_min
		target[53] = -0.0 + safety_buffer	#limit_x_neg_side

		target[54] = 0.0 - safety_buffer	#limit_x_pos_neutral
		target[55] = 0.0 - safety_buffer	#limit_x_pos_max
		target[56] = 0.0 - safety_buffer	#limit_x_pos_min
		target[57] = 0.0 - safety_buffer	#limit_x_pos_side

		target[58] = 5.000 #limit_x_pos_angle
		target[59] = 0.000 #limit_x_neg_angle
		target[60] = 10.000 #limit_y_pos_angle
		target[61] = -10.000 #limit_y_neg_angle

		target[62] = 10.000 #limit_z

	elif bin_num==4 or bin_num==6 or bin_num==7 or bin_num==9:
 		target[40] = 1	#bin_type
		target[41] = 0.250 - safety_buffer	#bin_size_x 
		target[42] = 0.225 - safety_buffer	#bin_size_y 
	
		target[43] = -0.105 + safety_buffer	#limit_y_up_suctionlift 
		target[44] = -0.250 - 0.015 + safety_buffer	#limit_y_up_suctionlower 
		target[45] = -0.105 + safety_buffer	#limit_y_up_suctionside

		target[46] = -0.0 + safety_buffer	#limit_y_do_suction
		target[47] = -0.0 + safety_buffer	#limit_y_do_grapmax 
		target[48] = -0.0 + safety_buffer	#limit_y_do_grapmin
		target[49] = -0.0 + safety_buffer	#limit_y_do_suctionside

		target[50] = -0.0 + safety_buffer	#limit_x_neg_neutral
		target[51] = -0.0 + safety_buffer	#limit_x_neg_max
		target[52] = -0.0 + safety_buffer	#limit_x_neg_min
		target[53] = -0.0 + safety_buffer	#limit_x_neg_side

		target[54] = 0.0 - safety_buffer	#limit_x_pos_neutral
		target[55] = 0.0 - safety_buffer	#limit_x_pos_max
		target[56] = 0.0 - safety_buffer	#limit_x_pos_min
		target[57] = 0.0 - safety_buffer	#limit_x_pos_side

		target[58] = 5.000 #limit_x_pos_angle
		target[59] = 0.000 #limit_x_neg_angle
		target[60] = 10.000 #limit_y_pos_angle
		target[61] = -10.000 #limit_y_neg_angle

		target[62] = 10.000 #limit_z

	elif bin_num==5 or bin_num==8:
 		target[40] = 1	#bin_type
		target[41] = 0.250 - safety_buffer	#bin_size_x
		target[42] = 0.225 - safety_buffer	#bin_size_y
	
		target[43] = -0.105 + safety_buffer	#limit_y_up_suctionlift
		target[44] = -0.250 - 0.015 + safety_buffer	#limit_y_up_suctionlower
		target[45] = -0.105 + safety_buffer	#limit_y_up_suctionside

		target[46] = -0.0 + safety_buffer	#limit_y_do_suction
		target[47] = -0.0 + safety_buffer	#limit_y_do_grapmax
		target[48] = -0.0 + safety_buffer	#limit_y_do_grapmin
		target[49] = -0.0 + safety_buffer	#limit_y_do_suctionside

		target[50] = -0.0 + safety_buffer	#limit_x_neg_neutral
		target[51] = -0.0 + safety_buffer	#limit_x_neg_max
		target[52] = -0.0 + safety_buffer	#limit_x_neg_min
		target[53] = -0.0 + safety_buffer	#limit_x_neg_side

		target[54] = 0.0 - safety_buffer	#limit_x_pos_neutral
		target[55] = 0.0 - safety_buffer	#limit_x_pos_max
		target[56] = 0.0 - safety_buffer	#limit_x_pos_min
		target[57] = 0.0 - safety_buffer	#limit_x_pos_side

		target[58] = 5.000 #limit_x_pos_angle
		target[59] = 0.000 #limit_x_neg_angle
		target[60] = 10.000 #limit_y_pos_angle
		target[61] = -10.000 #limit_y_neg_angle

		target[62] = 10.000 #limit_z
		
	#Offline Coefficient
	tx = 3	
	ttx = 0
	while tx < 20:
		ttx = ttx + target[tx]
		tx += 1
		
	target[20] = ttx/17

	return target, name

if __name__ == "__main__":
	print target
