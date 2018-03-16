#!/usr/bin/env python
#coding=utf-8

# some base libs
import numpy as np
import cv2, os, time, base64, urllib2, rospy
from json import *
# ros msg
from xbot_msgs.msg import FaceResult
from std_msgs.msg import String, UInt32

url = "http://192.168.8.141:8000/recognition"



class face_recog():
	"""docstring for face_recog"""
	def __init__(self):
		self.face_result_pub = rospy.Publisher('/office/face_result',FaceResult,queue_size=1)
		self.next_loop_sub = rospy.Subscriber('/office/next_loop',UInt32, self.next_loopCB)
		rospy.spin()




	def next_loopCB(self, loop_count):
		cap = cv2.VideoCapture(0)
		msg = FaceResult()
		if loop_count.data == 255:##got back to origin, start next loop
			while True:
				# Capture frame-by-frame
				ret, frame = cap.read()
				if ret:
					cv2.imwrite('tmp.jpg',frame)
				with open("tmp.jpg", "rb") as fp:
					image_binary = fp.read()
					image_binary = base64.b64encode(image_binary)
					post_data = {"Image":image_binary}
					body = JSONEncoder().encode(post_data)
					req = urllib2.Request(url, body)
					response = urllib2.urlopen(req).read()
					body = JSONDecoder().decode(response)
					rospy.loginfo(body)

				if body['Id'] == 'UNKNOWN' or body['Id'] == 'None':#no people in frame
					continue
				elif body['Confidence'] >0.6:#registered people in frame
					print body
					msg.is_staff = 1
					msg.name = body['Id']
					self.face_result_pub.publish(msg)
					break
				else:#unregistered people in frame
					msg.is_staff = 0
					msg.name = 'unknown'
					self.face_result_pub.publish(msg)
					break
		elif loop_count.data == 200:#unrecognized voice, reconfig if there still people talking to
			start_time = time.time()
			while True:
				# Capture frame-by-frame
				ret, frame = cap.read()
				if ret:
					cv2.imwrite('tmp.jpg',frame)
				with open("tmp.jpg", "rb") as fp:
					image_binary = fp.read()
					image_binary = base64.b64encode(image_binary)
					post_data = {"Image":image_binary}
					body = JSONEncoder().encode(post_data)
					req = urllib2.Request(url, body)
					response = urllib2.urlopen(req).read()
					body = JSONDecoder().decode(response)
					rospy.loginfo(body)

				if body['Id'] == 'UNKNOWN' or body['Id'] == 'None':#reconfiged nobody
					msg.is_staff = 0
					msg.name = 'nobody'
					self.face_result_pub.publish(msg)
					now_time = time.time()
					if now_time - start_time <= 3:
						continue
					else:
						break
				else:#reconfiged unregistered people
					msg.is_staff = 0
					msg.name = 'unknown'
					self.face_result_pub.publish(msg)
					break
		else:
			pass



if __name__ == '__main__':
	rospy.init_node('face_recog')
	try:
		rospy.loginfo('face recogition initialized...')
		face_recog()
	except rospy.ROSInterruptException:
		rospy.loginfo('face recogition initialize failed, please retry...')

