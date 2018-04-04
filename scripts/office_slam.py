#!/usr/bin/env python
#coding=utf-8

import rospy, yaml, os

from xbot_msgs.msg import FaceResult
from std_msgs.msg import String, UInt32
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty

class office_slam():
	"""docstring for office_slam"""
	def __init__(self):
		self.next_loop_pub = rospy.Publisher('/office/next_loop', UInt32, queue_size=1)
		self.goal_reached_pub = rospy.Publisher('/office/goal_reached', String, queue_size=1)
		self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.move_base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_resultCB)
		self.goal_name_sub = rospy.Subscriber('/office/goal_name', String, self.goal_nameCB)
		self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
		self.coll_position_dict = dict()
		self.current_goal = []
		self.loop_tag = UInt32()
		self.loop_tag.data = 0
		yaml_path = rospy.get_param('/office_slam/yaml_file_path','/home/roc/catkin_ws/src/xbot_navigoals')
		yaml_path = yaml_path + '/scripts/coll_position_dic.yaml'
		f = open(yaml_path, 'r')
		self.coll_position_dict = yaml.load(f)
		f.close()
		rospy.spin()


	def goal_nameCB(self, name):
		pos = self.coll_position_dict[name.data]
		goal = PoseStamped()
		goal.header.frame_id = 'map'
		goal.pose.position.x = pos[0][0]
		goal.pose.position.y = pos[0][1]
		goal.pose.position.z = pos[0][2]
		goal.pose.orientation.x = pos[1][0]
		goal.pose.orientation.y = pos[1][1]
		goal.pose.orientation.z = pos[1][2]
		goal.pose.orientation.w = pos[1][3]
		print goal
		self.move_base_goal_pub.publish(goal)
		self.current_goal = [name, goal]


	def move_base_resultCB(self, result):
		if result.status.status == 3:
			# success
			self.goal_reached_pub.publish(self.current_goal[0])
			if self.current_goal[0].data == 'origin':
				self.loop_tag.data = 255
				self.next_loop_pub.publish(self.loop_tag)
			self.clear_costmaps_srv()
		elif result.status.status == 4:
			#failed
			msg = String()
			msg.data = 'abort'
			self.goal_reached_pub.publish(msg)








if __name__ == '__main__':
	rospy.init_node('office_slam_serve')
	try:
		rospy.loginfo('office slam initialized...')
		office_slam()
	except rospy.ROSInterruptException:
		rospy.loginfo('office slam initialize failed, please retry...')
