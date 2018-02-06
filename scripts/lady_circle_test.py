#!/usr/bin/env python
#coding=utf-8
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class lady_circle():
	"""docstring for lady_circle"""
	def __init__(self):
		self.current_goal_num = 0
		goal1 = PoseStamped()
		goal2 = PoseStamped()
		self.goals = [goal1, goal2]
		reach_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.reach_goalCB)
		goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		goal_pub.publish(self.goals[self.current_goal_num%2])
		self.current_goal_num++

	def reach_goalCB(self, result):
		if result = 1:
			goal_pub.publish(self.goals[self.current_goal_num%2])
			current_goal_num++
		else:
			pass



if __name__ == '__main__':
	rospy.init_node('lady_circle')
	try:
		lady_circle()
		rospy.loginfo( 'lady for circle test initialized...')
	except rospy.ROSInterruptException:
		rospy.loginfo('lady circle initialize failed, please retry...')