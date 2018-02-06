#!/usr/bin/env python
#coding=utf-8

import time,cv2
import os, base64, urllib2
from json import *


url = "http://172.16.0.143:8000/recognition"

cap = cv2.VideoCapture(0)
total_time=0
test_num = 30
for i in xrange(1,test_num+1):
	ret, frame = cap.read()
	if ret:
		cv2.imwrite('tmp.jpg',frame)
	start=time.time()
	with open("tmp.jpg", "rb") as fp:
		image_binary = fp.read()
		image_binary = base64.b64encode(image_binary)
		post_data = {"Image":image_binary}
		body = JSONEncoder().encode(post_data)
		req = urllib2.Request(url, body)
		response = urllib2.urlopen(req).read()
		body = JSONDecoder().decode(response)
		print body
	end=time.time()
	total_time+=end-start
print total_time/test_num

cap.release()
cv2.destroyAllWindows()

#for TencentYoutuyun.conf.API_YOUTU_VIP_END_POINT end_point
#get four-character idioms
#retlivegetfour = youtu.livegetfour(session_id)
#print retlivegetfour

#four-character live detect without face compare
#retlivedetectfour = youtu.livedetectfour('1122', 'xxx.mp4', session_id)
#print retlivedetectfour

#four-character live detect with face compare
#retlivedetectfour= youtu.livedetectfour('1122',  'xxx.mp4',  session_id,   'xxx.jpg', True)
#print retlivedetectfour

#four-character idcard live detect
#retidcardlivedetectfour = youtu.idcardlivedetectfour('123456789987654321',  '张三',  '1122', 'xxx.mp4', session_id )
#print retidcardlivedetectfour

#idcard face compare: use local image compare with id card image 
#retidcardfacecompare = youtu.idcardfacecompare('123456789987654321', '张三', 'xxx.jpg', 0, session_id)
#print retidcardfacecompare

#idcard face compare :use url image compare with id card image
#retidcardfacecompare = youtu.idcardfacecompare('123456789987654321', '张三', 'http://xxx.png', 1, session_id)
#print retidcardfacecompare

# face compare : use two local image to compare 
#retfacecompare = youtu.FaceCompare('xxx.jpg', 'xxx.jpg')
#print retfacecompare

# face compare : use two url image to compare 
#retfacecompare = youtu.FaceCompare('http://xxx.png', 'http://xxx.png', 1)
#print retfacecompare

#id card ocr: use local id card image
#retidcardocr = youtu.idcardocr('xxx.jpg', data_type = 0, card_type = 0)
#print retidcardocr

#id card ocr: use url id card image
#retidcardocr = youtu.idcardocr('http://xxx.jpg', data_type = 1, card_type = 0)
#print retidcardocr

#driver license ocr: use local image
#retdriverlicenseocr = youtu.driverlicenseocr('ocr_xsz_01.jpg', data_type = 0, proc_type = 0)
#print retdriverlicenseocr

#business card ocr: use local image
#retbcocr = youtu.bcocr('ocr_card_01.jpg', data_type = 0)
#print retbcocr

#general ocr: use local image
#retgeneralocr = youtu.generalocr('icon_ocr_common01.png', data_type = 0)
#print retgeneralocr

#id card validate: validate the idcard is correct
#retvalidateidcard = youtu.ValidateIdcard('123456789987654321', '张三', session_id)
#print retvalidateidcard
