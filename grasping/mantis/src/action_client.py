#! /usr/bin/env python

import rospy
import math
import time

import actionlib
import mantis.msg

def a( state, result ):
    aa=""

def b():
    bb=""

def c( feedback ):
    d = feedback.current_status
    print d

def gripper_action_call( goal_sent ):

    client = actionlib.SimpleActionClient('APC_gripper_action', mantis.msg.GripperAction)
    client.wait_for_server()

    goal = mantis.msg.GripperGoal( item_id = goal_sent )
    client.send_goal( goal, a, b, c)
    client.wait_for_result()
    return client.get_result() 

if __name__ == '__main__':

	while not rospy.is_shutdown():
		print "Starting client!"
		goal_sent=["1.0 3.2 4.5 16.9 999.9 -0.5 1 0 0 0","124.0 -3259.2 -425.5 0.1259 9.19 -0.5 2 0 0 0","124.0 -3259.2 -425.5 0.1259 9.19 -0.5 3 0 0 0"]
		try:
			i = input("Send Goal Package...")
			rospy.init_node('main_program_manager')
			result = gripper_action_call( goal_sent[i-1] )
			print str(result)
		except rospy.ROSInterruptException:
			print "program interrupted before completion"




