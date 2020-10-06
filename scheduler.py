#!/usr/bin/env python
'''
Team id: 826
Author List: Amogh Gupta, Aditya Bucha 
Filename: demo_scheduler.py 
Theme: Survey and rescue 
Functions: distance, callbacks, rescueHoverTime, foodMedHoverTime, removeBeacons, estimatedTimeRescue
estimatedTimeFoodMed, upgradeRescueTime, handleDropoff, decideNext, decider 
'''
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import time 
import json 
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune

inf = int(1e9) #Infinity 

class sr_scheduler():

	def __init__(self):
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr',SRDroneStats, self.onboard_callback)	      
		rospy.Subscriber('/whycon/poses', PoseArray,self.position_callback) 
		rospy.Subscriber('/timer', Float64, self.timer_callback)  
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		self.rescueQueue = set() 
		self.foodMedQueue = set()                                                                     
		self.currJobInfo = None 
		self.currJobPos = None 
		self.jobTimer = 0 
		self.drone_position = [0]*3 
		self.baseStation = "E4"  
		self.food = None 
		self.meds = None     
		self.mutex = 0 #0 if free 1 if in use 
		self.vel = 2.6  

		#Load the coordinates of the cells 
		with open ("/home/aditya44/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json", "r") as readFile: 
			self.data = json.load (readFile)

	'''
	Function Name: distance
	Input: source and destination coordinates 
	Output:  distance between source and destination 
	Logic: euclidean distance
	Example call: self.distance (src, dest)   
	'''
	def distance (self, src, dest):
		s = 0 
		for i in range (3): 
			s += (src[i]-dest[i])**2 
		return s**0.5 

	def timer_callback(self, msg): 
		self.jobTimer = msg.data 

	'''
	Function Name: position_callback 
	Input: drone coordinates  
	Output: None 
	Logic: load the input into a class variable 
	Example call: self.position_callback(msg)  
	'''
	def position_callback(self, msg): 
		self.drone_position[0] = msg.poses[0].position.x 
		self.drone_position[1] = msg.poses[0].position.y 
		self.drone_position[2] = msg.poses[0].position.z  
		# print ("Current drone position: ", self.drone_position) 

	'''
	Function Name: onboard_callback 
	Input: stats  
	Output: None  
	Logic: load the onboard food and medicines into a class variable 
	Example call: self.onboard_callback(msg) 
	'''
	def onboard_callback (self, msg): 
		self.food = msg.foodOnboard 
		self.meds = msg.medOnboard 

	'''
	Function Name: rescueHoverTime 
	Input: beacon details (location, info, timestamp)  
	Output: estimated time to go and hover for rescue 
	Logic: use average velocity to compute reaching time and hovering time. Also included some extra margin 
	Example call: self.rescueHoverTime (beacon)   
	'''
	def rescueHoverTime (self, beacon):
		point = beacon[0] 
		#self.data has the coordinates for the given cell locations ('A1' etc)
		return (self.distance(self.data[point], self.drone_position)/self.vel) + 6   

	'''
	Function Name: foodMedHoverTime  
	Input: beacon details (location, info, timestamp)  
	Output: estimated time to go and hover for food/medicine 
	Logic: use average velocity to compute reaching time and hovering time. Also included some extra margin 
	Example call: self.foodMedHoverTime (beacon)   
	'''
	def foodMedHoverTime (self, beacon): 
		point = beacon[0] 
		service = beacon[1] 

		if (service == "FOOD" and self.food == 0) or (service == "MEDICINE" and self.meds == 0): 
			#Also include time to replenish the food/ medicine stock 
			return (self.distance (self.drone_position, self.data[self.baseStation])/self.vel) + 10 + (self.distance(self.data[self.baseStation], self.data[point]))  			
		else: 
			return self.distance (self.drone_position, self.data[point])/self.vel + 4  

	
	'''
	Function Name: estimatedTimeRescue 
	Input: None  
	Output: returns best possible rescue location; None if it's not possible to do any rescue  
	Logic: for all rescues, calculate the excess time the beacon would still be lit up after hovering  
	Example call: self.estimatedTimeRescue() 
	'''
	def estimatedTimeRescue (self):  
		#print ("In rescue")
		maxOffset = -1*inf 
		serviceLocation = None  

		for beacon in self.rescueQueue: 
			t = self.rescueHoverTime(beacon) 
			#Time the beacon would still be lit after hovering 
			offset = beacon[2] + 10 - time.time() - t #beacon[2] is the time stamp when the beacon was first detected  
	
			if maxOffset < offset:  
				maxOffset = offset
				serviceLocation = beacon[0] 

		if maxOffset > 0.5: #Some minimum threshold
			return (serviceLocation, "RESCUE")             
		
		return None  

	'''
	Function Name: estimatedTimeRescue 
	Input: None  
	Output: returns best possible food/medicine location; None if it's not possible to do any rescue  
	Logic: for all rescues, calculate the excess time the beacon would still be lit up after hovering  
	Example call: self.estimatedTimeFoodMed() 
	'''
	def estimatedTimeFoodMed(self): 

		#print ("IN food/med")
		maxOffset = -1*inf 
		serviceLocation = None 
		serviceType = None   

		
		for beacon in self.foodMedQueue: 
			t = self.foodMedHoverTime(beacon) 
			offset = beacon[2] + 30 - time.time() - t
	
			if maxOffset < offset:  
				maxOffset = offset
				serviceLocation = beacon[0] 
				serviceType = beacon[1] 

		if maxOffset > 0.5: #Some minimum threshold
			return (serviceLocation, serviceType)  

		return None 

	'''
	Function Name: upgradeRescueTime     
	Input: None 
	Output: returns if a rescue can be done midway another task; else None  
	Logic: if the drone if performing a food/medicine/replenish (hover over base for food/medicines) and a rescue comes up, choose if it's feasible
	Example call: self.upgradeRescueTime() 
	'''
	def upgradeRescueTime(self): 

		#print ("In upgrade")
		
		''' 
		Options: 
		1. Stop task and go to a rescue 
		2. continue with your task 
		'''  

		maxOffset = -1*inf 
		serviceLocation = None  

		#Compute the best possible rescue 
		for beacon in self.rescueQueue: 
			t = self.rescueHoverTime(beacon) 
			offset = beacon[2] + 10 - time.time() - t                    

			if maxOffset < offset:  
				maxOffset = offset
				serviceLocation = beacon[0] 
		
		#print (maxOffset) 
		if maxOffset > 0: #Possible to do a rescue 

			#Because a rescue anyway replenishes the supplies 
			if self.currJobInfo == "REPLENISH":   
				self.currJobInfo = "RESCUE" 
				return (serviceLocation, "RESCUE") 
			
			else: 
				remainingTime = 3 - self.jobTimer
				# print ("Best rescue", maxOffset, serviceLocation) 
				# remainingTime = 3 - self.jobTimer    
				if maxOffset - remainingTime < 0.8: #Condition                  
					self.currJobInfo = "RESCUE"
					return (serviceLocation, "RESCUE")  
		
		return None 
	
	'''
	Function Name: decideNext      
	Input: None 
	Output: returns the next job to perform. if nothing, return None 
	Logic: Written within finction 
	Example call: self.decideNext() 
	'''
	def decideNext(self): 

		'''
		self.currJobInfo has details of the job type
		1. RESCUE 
		2. FOOD
		3. MEDICINE
		4. REPLENISH- hover over base station to refill food and medicine 
		5. DROPOFF - hover over base station after rescue 
		6. None - no job at present 
		'''

		if self.currJobInfo is None: #Choose the best rescue. if not possible, choose best food/med 
			possibleRescue = self.estimatedTimeRescue() #Return rescue location for rescue beacon; else None  
			if possibleRescue is not None: 
				return possibleRescue 
			else: 
				possibleFoodMed = self.estimatedTimeFoodMed() #Return food/medicine location for other beacons; else None  
				return possibleFoodMed  
	
		#Cannot change 
		elif self.currJobInfo == "RESCUE" or self.currJobInfo == "DROPOFF":  
			return None 
		
		#Check if a rescue is now possible 
		else: 
			upgradeRescue = self.upgradeRescueTime() #Return a rescue location; else None  
			if upgradeRescue is not None: 
				return upgradeRescue		
			return None 
	
	'''
	Function Name: replenish      
	Input: None
	Output: None  
	Logic: publishes the decision to hover over the base station for supplies 
	Example call: self.replenish() 
	'''
	def replenish(self): 
		self.currJobInfo = "REPLENISH" 
		self.currJobPos = self.baseStation
		self.decision_pub.publish(self.baseStation, "BASE") 

	'''
	Function Name: decider      
	Input: None
	Output: None  
	Logic: publish the apt. decision based on the function  decisionNext  
	Example call: self.decider()  
	'''
	def decider(self): 
 
		#Mutex to stop redundant calls, because this function is called by detection and serviced callbacks and other functions
		if self.mutex == 1: 
			return 
		
		#print ("In decider") 
		self.mutex = 1 

		decided = self.decideNext()
		#print(decided)   
		if decided is not None:         
			if decided[1] == "FOOD" and self.food == 0 or decided[1] == "MEDICINE" and self.meds == 0: 
				self.replenish()  
			else:    
				self.currJobInfo = decided[1]     
				self.currJobPos = decided[0] 
				self.decision_pub.publish(decided[0], decided[1])          

		self.mutex = 0  

	'''
	Function Name: decision_callback      
	Input: beacon location and info 
	Output: None  
	Logic: add input decision to the job queue   
	Example call: self.detection_callback(msg) 
	'''
	def detection_callback(self, msg): 

		job = (msg.location, msg.info, time.time()) #Job format - (location, info, timestamp)  
		if msg.info == "RESCUE": 
			self.rescueQueue.add(job) 
		else: 
			self.foodMedQueue.add(job) 

		#Function to choose the next decision
		self.decider() 

	'''
	Function Name: handleDropoff       
	Input: None 
	Output: None  
	Logic: publilsh base station location after rescue                                   
	Example call: self.handleDropoff(msg)  
	'''
	def handleDropoff(self): 
		self.decision_pub.publish(self.baseStation, "BASE")   
		self.currJobInfo = "DROPOFF"  
		self.currJobPos = self.baseStation 


	'''
	Function Name: serviced_calllback        
	Input: msg location and success/failiure 
	Output: None  
	Logic:
	Remove the expired beacons and if successful and current job is rescue, go to basestation. 
	for other succesful messages, set the current job to none and call decider 
	for failures, and if location is the same as the current job location, set job to none and call decider                                  
	Example call: self.handleDropoff(msg)  
	'''
	def serviced_callback(self,msg):
		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc
		# print (msg.info, msg.location)

		point = msg.location 
		if point != self.baseStation: 

			toRem = None  
			for red in self.rescueQueue: 
				if red[0] == point: 
					toRem = red 
					break 
			try: 
				self.rescueQueue.remove(toRem)  
			except KeyError: 
				pass 

			if toRem is None: 
				for elem in self.foodMedQueue: 
					if elem[0] == point: 
						toRem = elem 
						break 
				try: 
					self.foodMedQueue.remove(toRem) 
				except KeyError: 
					pass    


		if msg.info == "SUCCESS": 
			#print ("SUCCESS",msg)                              
			if self.currJobInfo == "RESCUE": 
				self.handleDropoff()  

			else: 
				self.currJobInfo = None 
				self.decider()  
		else: 

			if msg.location == self.currJobPos: 
				#print ("Current job failed.", msg)  
				self.currJobInfo = None 
				self.decider()   

	def shutdown_hook(self):
		# This function will run when the ros shutdown request is recieved.
		# For instance, when you press Ctrl+C when this is running
		print ("Keyboard Interrupt.") 
		pass

def main(args):
	
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
