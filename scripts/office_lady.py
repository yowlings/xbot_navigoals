#!/usr/bin/env python
#coding=utf-8

import rospy, sys, termios, tty
import yaml

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionResult

class office_lady():
	"""docstring for office_lady"""
	def __init__(self):
		self.total_coll = input('please input total number of colleagues:\n')
		# print type(self.total_coll)
		# print self.total_coll
		self.num_coll = 0
		self.coll_position_dic = {}
		self.marker=Marker()
		self.marker.color.r=1.0
		self.marker.color.g=0.0
		self.marker.color.b=0.0
		self.marker.color.a=1.0
		self.marker.ns='office_lady'
		self.marker.scale.x=1
		self.marker.scale.y=0.1
		self.marker.scale.z=0.1
		self.marker.header.frame_id='map'
		self.marker.type=Marker.SPHERE_LIST
		self.marker.action=Marker.ADD
		self.arraymarker = MarkerArray()
		self.markers_pub = rospy.Publisher('/coll_position',MarkerArray,queue_size=1)
		self.goal_sub = rospy.Subscriber('/mark_coll_position',PoseStamped, self.mark_coll_positionCB)
		# self.goal_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_resultCB)
		# f = open('param/col_pos.yaml')
		# self.col_poses = yaml.load(f)
		# if col_poses is not NULL:
		# 	#push col_poses.pos to markerarray
		# else:
		# 	print 'please input the pose of every colleague with 2DNavi goal...'
		# 	print 'the first colleague is '+ col_poses.pose[0]
		
		tip = 'please input NO '+ str(self.num_coll) + 'colleague name:\n'
		self.name = input(tip)
		tip = 'please input the pose of NO ' + str(self.num_coll) +' with 2DNavi goal tool in rviz'
		print tip
		rospy.spin()

	def mark_coll_positionCB(self, pos):

		if self.num_coll < self.total_coll:
			self.coll_position_dic[self.name] = [[pos.pose.position.x,pos.pose.position.y,pos.pose.position.z],[pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z,pos.pose.orientation.w]]
			self.num_coll+=1
			print 'added '+str(self.num_coll)+' colleagues'
			print self.coll_position_dic
			self.marker.header.stamp =rospy.Time.now()
			self.marker.pose = pos.pose
			self.marker.id = self.num_coll - 1
			self.arraymarker.markers.append(self.marker)
			self.markers_pub.publish(self.arraymarker)
			if self.num_coll < self.total_coll:
				tip = 'please input NO '+ str(self.num_coll) + 'colleague name:\n'
				self.name = input(tip)
				tip = 'please input the pose of NO' + str(self.num_coll) +' with 2DNavi goal tool in rviz'
				print tip
			else:
				f=open('coll_position_dic.yaml', 'w')
				yaml.dump(self.coll_position_dic, f)
				f.close()
				print 'all colleagues position are marked and saved, please press ctrl+c to exit... '
	def goal_resultCB(self, result):
		pass






if __name__ == '__main__':
	rospy.init_node('office_lady_serve')
	try:
		rospy.loginfo('office lady initialized...')
		office_lady()
	except rospy.ROSInterruptException:
		rospy.loginfo('office lady initialize failed, please retry...')