#! /usr/bin/env python
import rospy
import actionlib
import demo.msg

class Gripper_action_server(object):
  _feedback = demo.msg.GripperFeedback()
  _result   = demo.msg.GripperResult()

  def __init__(self, name):
    self.server = actionlib.SimpleActionServer('APC_gripper_action', demo.msg.GripperAction, execute_cb=self.task, auto_start = False)
    self.server.start()
    
  def task(self, goal):

    item_no=""
    gripper_mode=""
    item_no,gripper_mode= goal.item_id.split(',')   

    print item_no
    print gripper_mode    

    ######----Open/Close of gripper-----#########
    if gripper_mode=='1':           ####   Open the gripper when received '1'

      #######################################
      ######   open the gripper   ###########
      #######################################


      #######################################

      self._feedback.current_status = "Gripper Open!"
      self.server.publish_feedback(self._feedback)
    elif gripper_mode=='2':          ####   Open the gripper when received '2'

      #######################################
      ######   Closing the gripper   ########
      #######################################


      #######################################

      self._feedback.current_status = "Gripper Closed!"
      self.server.publish_feedback(self._feedback)
   
 
      #################################################
      ######   To check whether item dropped   ########
      #################################################
      item_dropped = True
      if item_dropped==True:      # only sent when item dropped
        self._feedback.current_status = "dropped!!!!"
        self.server.publish_feedback(self._feedback)

      #################################################
    
    success=True
    if success:
      self._result.task_status = True
      rospy.loginfo('Action Succeeded!!')
      self.server.set_succeeded(self._result)


if __name__ == '__main__':
  print "Starting Action Server!"
  rospy.init_node('gripper_node')
  Gripper_action_server(rospy.get_name())
  rospy.spin()
