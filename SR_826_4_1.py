#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
import imutils
import copy

class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img = None 
		self.red = None 
		self.blue = None 
		self.green = None 
		self.contours = {}
		self.alreadyRed = set() 
		self.alreadyGreen = set() 
		self.alreadyBlue = set()  

	def load_rois(self, file_path = 'rect_info.pkl'):
		try:
			infile = open('/home/aditya44/catkin_ws/src/survey_and_rescue/scripts/contourPositions','rb') 
			# self.rect_list = pickle.load(infile)
			self.contours = pickle.load(infile)
			infile.close()   
		except IOError, ValueError:
			print("File doesn't exist or is corrupted")

 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)

 	def serviced_callback(self, msg):
 		pass
 	
	def detect_color_contour_centers(self):
		imgCopy = self.img.copy() 
		hsv = cv2.cvtColor (imgCopy, cv2.COLOR_BGR2HSV) 

		lower_red = np.array([0,150,120])
		upper_red = np.array([5,255,255])
		mask1 = cv2.inRange(hsv, lower_red, upper_red)
		lower_red = np.array([175,150,120])
		upper_red = np.array([180,255,255])
		mask2 = cv2.inRange(hsv,lower_red,upper_red)
		redMask = mask1+mask2

		lower_blue = np.array([110,180,120])
		upper_blue = np.array([130,255,255])
		blueMask= cv2.inRange(hsv,lower_blue,upper_blue) 

		lower_green = np.array([65,120,120])
		upper_green = np.array([80,255,255]) 
		greenMask = cv2.inRange(hsv,lower_green,upper_green)  

		blueOutput = cv2.bitwise_and(imgCopy, hsv, mask=blueMask)   
		redOutput = cv2.bitwise_and(imgCopy, hsv, mask=redMask) 
		greenOutput = cv2.bitwise_and(imgCopy, hsv, mask=greenMask) 

		self.red = cv2.findNonZero(redMask) 
		self.blue = cv2.findNonZero(blueMask) 
		self.green = cv2.findNonZero(greenMask) 

		if self.red is None: 
			self.red = [] 
		if self.blue is None: 
			self.blue = []  
		if self.green is None: 
			self.green = [] 

	def check_whether_lit(self):
		
		def getPosition (Contour,point): 
			dist = cv2.pointPolygonTest (Contour, point, False) 
			if dist>=0:
				return True
			else: 
				return False 
		
		redPoints,bluePoints,greenPoints = set(),set(),set() 
		for point in self.red: 
			x,y = point[0][0],point[0][1] 
			for contour in self.contours.keys():  
				res = getPosition (self.contours[contour], (x,y)) 
				if res: 
					redPoints.add(contour)
					break  
    		for point in self.blue:  
			x,y = point[0][0],point[0][1] 
			for contour in self.contours.keys():  
				res = getPosition (self.contours[contour], (x,y)) 
				if res:  
					bluePoints.add(contour) 
					break 
		for point in self.green: 
			x,y = point[0][0],point[0][1] 
			for contour in self.contours.keys():  
				res = getPosition (self.contours[contour], (x,y)) 
				if res: 
					greenPoints.add(contour)
					break   
		

		for point in redPoints: 
			if point not in self.alreadyRed: 
				#publish 
				self.detect_pub.publish (point, "RESCUE") 
				
		for point in bluePoints: 
			if point not in self.alreadyBlue: 
				#publish 
				self.detect_pub.publish (point, "MEDICINE") 

		for point in greenPoints: 
			if point not in self.alreadyGreen: 
				#publish 
				self.detect_pub.publish (point, "FOOD") 

		self.alreadyRed = redPoints 
		self.alreadyBlue = bluePoints 
		self.alreadyGreen = greenPoints 

		

def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False) 
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(30)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.detect_color_contour_centers()
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)