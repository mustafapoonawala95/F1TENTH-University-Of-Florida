#!/usr/bin/env python

from __future__ import print_function
import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from race.msg import pid_input
import math
import numpy as np
import sys
#from os import system, name 
from time import sleep

KP = 20
KD = 0.01
limit_ang = 20
prev_error = 0.0
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
def control_callback(data):
	global prev_error
	global limit_ang
	velocity = data.pid_vel
	error = 5*data.pid_error
	angle = 0.0

	if error!=0.0:
		control_error = KP*error + KD*(error-prev_error)
		angle = angle + control_error*np.pi/180

	prev_error = error

	# Turn Limits
	if angle > limit_ang*np.pi/180:
			angle = limit_ang*np.pi/180
	if angle < -limit_ang*np.pi/180:
			angle = -limit_ang*np.pi/180

	msg = drive_param()

	# Publish drive_param message
	msg.velocity = velocity
	msg.angle = angle
	pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", pid_input, control_callback)
	rospy.spin()

