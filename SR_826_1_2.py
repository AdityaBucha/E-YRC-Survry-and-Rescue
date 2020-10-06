#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self): 
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0] 

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = None # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		# self.Kp = [0,0,0]
		# self.Ki = [0,0,0]
		# self.Kd = [0,0,0]


		#initial setting of Kp, Kd and Ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [1179*0.06,546*0.06,568*0.06]
		self.Ki = [44*0.008,17*0.008,17*0.008]
		self.Kd = [1790*0.3,1048*0.3,1441*0.3]


		#-----------------------Add other required variables for pid here ----------------------------------------------


		self.prevError = [0,0,0]
		self.Iterm = [0,0,0] 
		self.out_roll = 0
		self.out_pitch = 0
		self.out_throttle = 0
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.sample_time = 0.060 # in seconds




		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_pub = rospy.Publisher('/alt_error', Float64, queue_size=1) 
		self.pitch_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1) 
		self.roll_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)  





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y 
		self.drone_position[2] = msg.poses[0].position.z  
		
		

		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------


	def pitch_set_pid(self,pit): 
		self.Kp[1] = pit.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pit.Ki * 0.008
		self.Kd[1] = pit.Kd * 0.3


	def roll_set_pid(self,rol): 
		self.Kp[0] = rol.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = rol.Ki * 0.008
		self.Kd[0] = rol.Kd * 0.3 


	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


		#Computed error
		error = [0,0,0]   
		for i in range (3): 
			error[i] = self.drone_position[i] - self.setpoint[i]  
		
		diffError = [0,0,0] 
		for i in range (3):
			diffError[i] = error[i] - self.prevError[i]
		
		#pid output 
		output = [0,0,0] 
		for i in range (3): 
			output[i] = self.Kp[i]*error[i] + self.Iterm[i] + self.Kd[i]*(error[i] - self.prevError[i]) 

		self.out_roll = output[0] 
		self.out_pitch = output[1] 
		self.out_throttle = output[2] 

		#Update roll, pitch and throttle 
		self.cmd.rcRoll = 1500 - self.out_roll # by trial and error  
		self.cmd.rcPitch = 1500 + self.out_pitch
		self.cmd.rcThrottle = 1500 + self.out_throttle

		
		#Limiting values 
		if self.cmd.rcRoll > self.max_values[0]: 
			self.cmd.rcRoll = self.max_values[0] 
		if self.cmd.rcRoll < self.min_values[0]: 
			self.cmd.rcRoll = self.min_values[0]

		if self.cmd.rcPitch > self.max_values[1]: 
			self.cmd.rcPitch = self.max_values[1] 
		if self.cmd.rcPitch < self.min_values[1]: 
			self.cmd.rcPitch = self.min_values[1]

		if self.cmd.rcThrottle > self.max_values[2]: 
			self.cmd.rcThrottle = self.max_values[2] 
		if self.cmd.rcThrottle < self.min_values[2]: 
			self.cmd.rcThrottle = self.min_values[2]
	

		#Update the previous error
		#Equivalent to deepcopy 
		for i in range (3): 
			self.prevError[i] = error[i] 
		
		#Updating the Iterm
		for i in range (3): 
			self.Iterm[i] = (self.Iterm[i] + error[i])*self.Ki[i]  	
		

	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)
		self.alt_pub.publish(self.out_throttle)
		self.pitch_pub.publish(self.out_pitch)
		self.roll_pub.publish(self.out_roll)
		

def hasReached(dest, curr): #Returns True if drone is within acceptable error (0.2, 0.2, 0.5) 

	if abs(dest[0]-curr[0])<=0.2 and abs(dest[1]-curr[1])<=0.2 and abs(dest[2]-curr[2])<=0.5: 
		return True 
	else: 
		return False  


if __name__ == '__main__':

	targets = [
				[0.2, 0, 23], [3.5, -3.3, 23], [ -4.4, -4.6, 20.7] , 
				[-6.0, 6.3, 18.3], [5.0, 5.3, 16.3]
			] # 2-D list to hold the target positions 


	e_drone = Edrone()
	r = rospy.Rate(17) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz

	for i in range (len(targets)-1): 
		
		e_drone.setpoint = targets[i]
		while True: 

			if not hasReached(e_drone.setpoint, e_drone.drone_position): #execute pid algorithm until the drone is within acceptable error
				e_drone.pid()
				r.sleep() 
			
			else:
				print'Reached level {}'.format(i+1) 
				break 


	n = len(targets) 
 	e_drone.setpoint = targets[n-1]  
	
	while not rospy.is_shutdown(): #Drone should finally settle at the final destination

		if hasReached(e_drone.setpoint, e_drone.drone_position):
			print("path completed")  
			
		e_drone.pid()
		r.sleep()