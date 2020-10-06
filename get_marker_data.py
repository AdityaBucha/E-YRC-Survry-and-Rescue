#!/usr/bin/env python

import rospy
import roslib
import tf 

from geometry_msgs.msg import PoseArray

#Defining a class
class Marker_detect():

	def __init__(self):
	 	rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
			
		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		 

	# Callback for /whycon/poses
	# Please fill in the function
	def whycon_data(self,msg):

		# Printing the detected markers on terminal
		print(self.whycon_marker) 
		
		points = msg.poses 
		for i in range (len(points)): 
			x = points[i].position.x    
		 	y = points[i].position.y 
			z = points[i].position.z
			self.whycon_marker.setdefault(i, list([x,y,z])) 

if __name__=="__main__":
	marker = Marker_detect()
	while not rospy.is_shutdown():
		rospy.spin()
