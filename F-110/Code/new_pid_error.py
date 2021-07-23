#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from race.msg import pid_input
import pdb
import os
from time import sleep
flag=False
desired_trajectory = 0.30

base_vel = 1.0
vel_scale = 1.0

pub = rospy.Publisher('pid_error', pid_input, queue_size=10)
#pub1 = rospy.Publisher('pid_error2', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 3.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0


# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, theta):
 	# TODO: implement
	if math.isnan(theta) or math.isinf(theta):
		print ('encountered invalid value in getRange')
		theta = 0.0
	if theta < 0.0: theta = 0.0
	if theta > 180.0: theta = 180.0

	idx_float = ((theta+45.0) / 270.0) * (len(data.ranges) - 1)
	idx = int(round(idx_float))
	ret = data.ranges[idx]
	return ret if not math.isnan(ret) and not math.isinf(ret) else 3.0


# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
	# TODO: implement
	vel1 = 1
	
	#try1=np.asarray(data)
	dist=[]
	for angles in range(90,180):
		dist.append(getRange(data,angles))


	a = min(dist)
	theta=dist.index(a)+40
	b = dist[theta]
	
	
	
	"""
	swing = math.radians(theta)

	alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
	dist_AB = b * math.cos(alpha)

	dist_AC = vel1 * 0.1
	dist_CD = dist_AB + dist_AC * math.sin(alpha)
	"""

	error = desired_distance - a

	print('\r','Dist theta: %f \t Dist 0: %f' %(b,a),end = " ")
	print('\t car distance: ', a)
	if math.isnan(error):
		print ('nan occured:', a, b)

	vel = base_vel * 1.0 / (vel_scale * abs(error) + 1)
	return (-error), vel


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
	# TODO: implement
	vel1 = 1
	dist=[]
	for angles in range(0,90):
		dist.append(getRange(data,angles))


	a = min(dist)
	theta=dist.index(a)+40
	b = dist[theta]

	"""
	if(c<=0.75 or flag==True):
	
		front_error=0.45
		flag=True
	
	if(c>=1.25 and flag==True):
		flag=False
		front_error=0
	
	
	swing = math.radians(theta)

	alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
	dist_AB = b * math.cos(alpha)
	
	dist_AC = vel1 * 0.1
	dist_CD = dist_AB + dist_AC * math.sin(alpha)
	"""
	error = desired_distance - a
	
	print('\r','Dist theta: %f \t Dist 0: %f' %(b,a),end = " ")
	print('\t car distance: ', a)
	if math.isnan(error):
		print ('nan occured:', a, b)

	vel = base_vel * 1.0 / (vel_scale * abs(error) + 1)
	return error, vel

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
	# TODO: implement
	left=180
	right=0
	dist=[]
	for angles in range(-45,225):
		dist.append(getRange(data,angles))
	min_dist=min(dist)
	min_index=dist.index(min_dist)
	if(min_index<=145 and min_index>=125): #This case if for the wall in front or any obstacle in front
		#We are gonna test two cases here: which are to make sure if there is a wall on left or right close to the car.
		error, vel =go_90(data,min_dist)

	elif(min_index<=55 and min_index>=0):
		error,vel=followRight(data,desired_trajectory)

	elif(min_index>=215 and min_index<270):
		error,vel=followLeft(data,desired_trajectory)

	return error,vel

def go_90(data,distance):
	left=180
	right=0
	if(distance<0.5):
		vel=-1 #reverse algorithm
		error=0
		return error,vel
	if(getRange(data,left)<getRange(data,right) and getRange(data,left)<2):
		error,vel=followLeft(data,desired_trajectory)
	elif(getRange(data,left)>getRange(data,right) and getRange(data,right)<2):
		error,vel=followRight(data,desired_trajectory)
	else:
	return error,vel
# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):

	desired_distance = 0.30  #The distance entere here will be increased by 0.15 further in the algo so, 0.15 means 0.3
	#Right follow function
	#error, vel = followRight(data, desired_distance)
	error, vel = followLeft(data, desired_distance)
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel

	pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!

if __name__ == '__main__':
	print("entering the pid_error mode",)
	#base_vel = float(sys.argv[1]) if len(sys.argv) > 1 else base_vel
	#vel_scale = float(sys.argv[2]) if len(sys.argv) > 2 else vel_scale
	#vel = base_vel

	rospy.init_node('second_pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()


