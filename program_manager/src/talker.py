#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('UR_target_pose', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
	#hello_str="halo!!"
        pub.publish(hello_str)
	rospy.loginfo("published!!")
        rate.sleep()

if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    

'''
if __name__ == '__main__':

	msg = "p[-0.256262,0.328373,0.562112,-0.630223,-1.47189,0.654198]"
	print msg
	pose=[ 0, 0, 0, 0, 0, 0]
	pose_test=[ '','' ,'' ,'' ,'' ,'' ]
	test,test_pose=msg.split('[')
	print test_pose
	msg,test_pose2=test_pose.split(']')
	print msg
	pose_test[0],pose_test[1],pose_test[2],pose_test[3],pose_test[4],pose_test[5]=msg.split(',')
	for i in range (6):
		pose[i]=float(pose_test[i])
	print pose

'''






	
	
