#!/usr/bin/env python
#coding=utf-8

import time,cv2
import TencentYoutuyun

# pip install requests
# please get these values from http://open.youtu.qq.com
appid = '10115085'
secret_id = 'AKIDqa7SblceJnALK7CIdoodXKwMflfOQLXG'
secret_key = 'gCYqwqZ5PXTEqHVXREuV7Yn5C7qUq8KL'
userid = 'xbot_face'

#choose a end_point
#end_point = TencentYoutuyun.conf.API_TENCENTYUN_END_POINT
#end_point = TencentYoutuyun.conf.API_YOUTU_VIP_END_POINT 
end_point = TencentYoutuyun.conf.API_YOUTU_END_POINT
youtu = TencentYoutuyun.YouTu(appid, secret_id, secret_key, userid, end_point)
session_id = "zkzdzn"

cap = cv2.VideoCapture(0)

while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()

	# Our operations on the frame come here
	# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Display the resulting frame
	cv2.imshow('frame',frame)
	if cv2.waitKey(1) & 0xFF == ord('r'):
		cv2.imwrite('tmp.jpg',frame)
		tip = 'please input your name in pinyin:\n'
		name = input(tip)
		person_id = input('please input your person id:\n')
		register_result = youtu.NewPerson(person_id,'tmp.jpg',['zkzdzn'],name)
		print register_result
	elif cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


