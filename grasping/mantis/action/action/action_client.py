#! /usr/bin/env python

import rospy
import actionlib
import demo.msg

def a(state,result):
    aa=""

def b():
    bb=""

def c(feedback):
    d = feedback.current_status
    print d


def gripper_action_call(goal_sent):

    client = actionlib.SimpleActionClient('APC_gripper_action', demo.msg.GripperAction)
    client.wait_for_server()

    goal = demo.msg.GripperGoal(item_id=goal_sent)

    client.send_goal(goal,a,b,c)

    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':

    print "Starting client!"
    goal_sent=["88,1","88,2"]

    for i in range (2):
        try:
            rospy.init_node('main_program_manager')
            result = gripper_action_call(goal_sent[i])
            print str(result)
            
        except rospy.ROSInterruptException:
            print "program interrupted before completion"





