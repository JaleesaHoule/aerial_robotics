#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy, actionlib
import rospkg
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

# standard libraries
import numpy as np
import copy
#from scipy.spatial.transform import Rotation as R
################################################################################


'''
The goal of this node is to read in position+orientation error values between the drone and Apriltag, calculate appropriate control gains to minimize the error, publish those values to the drone to control it in an attempt to land. Once the drone gets close enough to the landing pad, with zero x,y error, it can initiate self landing protocol.

Process:

-> first, initialize mission with ./execute_mission.sh 
-> rosrun robowork_minihawk_landingmission mission.py

within this node:
-> check for tag_detections -> while detections is none: wait
-> onces detections are being published:
	- get pose + orientation messages
	- calculate gains via PID controller to minimize error
	- change to qloiter mode + start to send rc commands with calculated gains
	- repeat until altitude is low enough that tag detections are lost
-> once the drone is close to the landing pad with small (x,y) error, can then initiate landing protocol


'''


class MinihawkController(object):
	def __init__(self):

		self.flightmode = 'AUTO'
		self.position = None
		self.orientation = None
		self.prior_position = None
		self.prior_orientation = None
		self.position_history  = []
		self.orientation_history  = []
		self.integral_error = {'roll': [0], 'pitch': [0],  'throttle': [0],  'yaw': [0]  }
		self.tag_detection= False
		self.control_publisher = rospy.Publisher("/minihawk_SIM/mavros/rc/override", OverrideRCIn, queue_size=10)
		self.dt = .1	
        	rospy.init_node("robowork_minihawk_landingmission")
        	print("Initializing ros node")
		self.module_rate = rospy.Rate(1/self.dt)
		
    	def update_flight_mode(self):
		last_flight_mode = self.flightmode
		if self.tag_detection == False and (self.position is None):
			self.flightmode= 'AUTO'

		elif self.tag_detection == True:
			self.flightmode = 'QLOITER'
		elif (self.tag_detection == False) and (self.position is not None) and (self.position.z < 10):
     			self.flightmode = 'QLAND'
			print('Met landing criteria')
		
		# only need to update controller flight mode if condition changes
		if last_flight_mode != self.flightmode:
			mode_publisher = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
			updatedmode = mode_publisher(custom_mode= self.flightmode)
			print('Changed flight mode to ', str(self.flightmode))


	def convert_quats_to_euler(self, quats):

		w = quats.w
		x = quats.x
		y = quats.y
		z = quats.z

		t0 = 2 * (w * x + y * z)
		t1 = 1 - 2* (x * x + y**2)
		X = np.arctan2(t0, t1)
		
		t2 = 2 * (w * y - z * x)
		t2 = 1 if t2 > 1 else t2
		t2 = -1 if t2 < -1 else t2
		Y = np.arcsin(t2)

		t3 = 2 * (w * z + x * y)
		t4 = 1 - 2 * (y**2 + z * z)
		Z = np.arctan2(t3, t4)

		return {"x":X, "y": Y,"z": Z}


	def get_apriltag_values(self,tagInfo):
		try:
			self.position= tagInfo.detections[0].pose.pose.pose.position
			self.orientation = self.convert_quats_to_euler(tagInfo.detections[0].pose.pose.pose.orientation)
			self.tag_detection=True
			if self.prior_position==self.position:
				self.tag_detection = False
		except:
			self.tag_detection = False

		

	def control_drone_movement(self):
			# calculate controls based on self.position & self.orientation: 4 first channels are 0:roll(-left,+right), 1:pitch(-up,+down), 2:throttle(-down,+up), 3:yaw(-left,+right))] standard neutral values: [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			
			roll_gain, pitch_gain, throttle_gain, yaw_gain = self.get_all_controls()

			new_channel_values=  [1500 + roll_gain, 1500 + pitch_gain, 1500 + throttle_gain, 1500 + yaw_gain, 1800, 1000 , 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

			self.control_publisher.publish(channels= new_channel_values)
			print('updated QLOITER rc controls: ', new_channel_values)


	def PID(self, current_error, prior_error,integral_error_, Kp, Kd, Ki, printcontrol, setpoint=0, ):

		error =  setpoint - current_error
		integral_error_.append(error*self.dt)

		kp_ = Kp * error
		kd_ = Kd * (current_error - prior_error)/self.dt
		ki_ = Ki *np.sum(integral_error_[-15:])

		gain = kp_ + kd_ + ki_ 
		print(printcontrol, ': ', 'Kp:', kp_, 'Ki:', ki_, 'Kd:', kd_, 'total gain:', gain) 

		
		#use max gain of 100
		if np.abs(gain) > 100:
			gain = np.sign(gain)*100
		
		return gain


	def get_all_controls(self):

		try:
			roll_gain = self.PID(self.position.x, self.prior_position.x, self.integral_error['roll'],5, 10, 1, 'roll')
			pitch_gain= self.PID(self.position.y, self.prior_position.y, self.integral_error['pitch'],5, 10, 1, 'pitch')
			throttle_gain = self.PID(self.position.z, self.prior_position.z,self.integral_error['throttle'],5,10,1, 'throttle')
			yaw_gain = self.PID(self.orientation.get('z'), self.prior_orientation.get('z'),self.integral_error['yaw'],1,10,1, 'yaw')
			return roll_gain, pitch_gain, throttle_gain, yaw_gain
		except: 
			return 0,0,0,0
		
	def main(self):

        	
		while not rospy.is_shutdown():

			# get current error values 
			rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, self.get_apriltag_values)
			print('******************************************* \n\n April tag detected?', self.tag_detection)
			self.position_history.append(self.position)
			self.orientation_history.append(self.orientation)

			if len(self.position_history) > 3:
				self.prior_position = self.position_history[-2]
				self.prior_orientation = self.orientation_history[-2]
				print('Current Error:', self.position,) #'prior error:', self.prior_position, 'same?', (self.prior_position==self.position))
				self.position_history.pop(0)
				self.orientation_history.pop(0)
			
			# delete old values to save memory
			#if len(self.integral_error) > 20:
			#	self.integral_error.pop(0)


			# check + update flight mode as needed
			self.update_flight_mode()

			# minimize error to help drone land 
			if self.flightmode =='QLOITER':
				self.control_drone_movement()
			else: 
				print('Currently in ', self.flightmode)

			self.module_rate.sleep()


           	#rospy.spin()



        
################################################################################

if __name__ == '__main__':    

    MinihawkController().main()
