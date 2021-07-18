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
#from os import system, name 
from time import sleep

pub = rospy.Publisher('pid_error', pid_input, queue_size=10)

base_vel = 1.0
vel_scale = 1.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, theta):
	if math.isnan(theta) or math.isinf(theta):
		print ('encountered invalid value in getRange')
		theta = 0.0
	if theta < 0.0: theta = 0.0
	if theta > 180.0: theta = 180.0

	idx_float = ((theta+45.0) / 270.0) * (len(data.ranges) - 1)
	idx = int(round(idx_float))
	ret = data.ranges[idx]
	return ret if not math.isnan(ret) and not math.isinf(ret) else 10.0

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.

def followLeft(data, desired_distance):
	theta = 140
	theta_for_swing=40
	vel = 0.2
	dist=[]
	for angles in range(85,95):
		dist.append(getRange(data,angles))

	d = getRange(data,theta)
	e = getRange(data,180)
	c = min(dist)#Front Distance

	swing = math.radians(theta_for_swing)

	alpha = math.atan((d * math.cos(swing)-e) / (d * math.sin(swing)))

	dist_AB = e * math.cos(alpha)
	
	dist_AC = vel * 0.1
	dist_CD = dist_AB + dist_AC * math.sin(alpha)
	if(dist_CD<c):

		error = desired_distance - dist_CD
		print("left ditance")
	
	else:# Front Distance error if wall is close
		error = desired_distance - c 
		print("Front ditance")

	print('\r','Car Dist left: %.4f\tError: %.4f\tCar Dist Front: %.4f' %(e,error,c))
	#sleep(0.1)
	#_ = system('clear')

	if math.isnan(error):
		print ('nan occured:', d, e)

	return (-error), vel

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
	theta = 40
	vel = 0.5
	theta_for_swing=40
	dist=[]
	for angles in range(85,95):
		dist.append(getRange(data,angles))

	a = getRange(data,theta)
	b = getRange(data,0)
	c = min(dist)#Front Distance


	swing = math.radians(theta)
	
	alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
	dist_AB = b * math.cos(alpha)
	
	dist_AC = vel * 0.1
	dist_CD = dist_AB + dist_AC * math.sin(alpha)
	if(dist_CD<c):
		error = desired_distance - dist_CD
		print("Right ditance")
	else:# Front Distance error if wall is close
		error = desired_distance - c
		print("Front ditance")

	print('\r','Car Dist Right: %.4f\tError: %.4f\tCar Dist Front: %.4f' %(dist_CD,error,c))
	#sleep(0.1)
	#_ = system('clear')

	if math.isnan(error):
		print ('nan occured:', a, b)

	return error, vel
# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def avoid_dist(data,desired_distance):
	theta = 40
	theta_left=140
	vel = 0.5
	dist=[]
	for angles in range(85,95):
		dist.append(getRange(data,angles))

	a = getRange(data,theta)
	b = getRange(data,0)
	c = min(dist)#Front Distance
	d = getRange(data,theta_left)
	e = getRange(data,180)
	swing = math.radians(theta)

	alpha_left=math.atan((d*math.cos(swing)-e) / (d * math.sin(swing)))

	alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
	dist_AB = b * math.cos(alpha)

	dist_AB_left= e * math.cos(alpha_left)
	
	dist_AC = vel * 0.1
	dist_CD = dist_AB + dist_AC * math.sin(alpha)

	dist_CD_left = dist_AB_left + dist_AC * math.sin(alpha_left)

	if(dist_CD<dist_CD_left):
		if(dist_CD<c):
			
			error = desired_distance - dist_CD
			print("Right distance")
		else:
			error = desired_distance - c
			print("Front Right ditance")
	else:# Front Distance error if wall is close
		if(dist_CD_left<c):
			error = dist_CD_left - desired_distance 
			print("Left distance")
		else:
			error = c - desired_distance 
			print("Front Left ditance")
    
	print('\tDist Right: %.4f\tDist Front: %.4f\tDist left: %.4f\tError: %.4f' %(dist_CD,c,dist_CD_left,error))
	#sleep(0.1)
	#_ = system('clear')

	if math.isnan(error):
		print ('nan occured:', a, b)

	return error, vel
  	#return 0.0

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):

	desired_distance = 2.5
	#error, vel = followRight(data, desired_distance)
	#error, vel = followLeft(data, desired_distance)
	error,vel = avoid_dist(data, desired_distance)
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel

	pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
