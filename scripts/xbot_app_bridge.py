#!/usr/bin/env python
#coding=utf-8

import rospy,numpy
from xbot_msgs.msg import XbotPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from tf.transformations import *
# clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
# resp1 = clear_costmaps_srv()

class xbot_app_bridge():
	"""docstring for xbot_app_bridge"""
	def __init__(self):
		self.amcl_pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_poseCB)
		self.xbot_pose = rospy.Publisher('/app/xbot_pose', XbotPose, queue_size=10)
		self.orientation_quat = Quaternion()
		rospy.spin()



	def amcl_poseCB(self,msg):
		self.orientation_quat = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w)
		euler = euler_from_quaternion(self.orientation_quat)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		pose_msg = XbotPose()
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.x = msg.pose.pose.position.x
		pose_msg.y = msg.pose.pose.position.y
		pose_msg.theta = yaw
		self.xbot_pose.publish(pose_msg)



if __name__ == '__main__':
	rospy.init_node('xbot_app_bridge')
	try:
		rospy.loginfo('office slam initialized...')
		xbot_app_bridge()
	except rospy.ROSInterruptException:
		rospy.loginfo('office slam initialize failed, please retry...')


