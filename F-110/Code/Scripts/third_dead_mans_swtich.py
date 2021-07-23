#!/usr/bin/env python

import rospy
import pdb
import numpy as np
import tf
import os
import time
import sys, select, termios, tty
from race.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header

pub = rospy.Publisher("vesc/high_level/ackermann_cmd_mux/input/nav_0",AckermannDriveStamped,queue_size = 10)

JOY_TIMER_PERIOD = 100000000   # 100 million nanoseconds -> 0.1 seconds
JOY_INIT_DELAY = 5             # 5 seconds


key ='m'
var1=0
rospy.init_node("dead_mans_switch")

def timer_callback(event):
	global joystick_present
	global key

	#This is the code for the keyboard keys.
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	joy_timer = rospy.timer.Timer(rospy.Duration(0, JOY_TIMER_PERIOD), timer_callback, oneshot=True)
    
# Need to initialize joy_timer twice: once at startup (here), and another time in the callback.
# Reason: the time.sleep() call introduces delay and might lead to the callback being called again when
# it should be sleeping. This is also why we must use a one-shot timer instead of a repeating one.

joy_timer = rospy.timer.Timer(rospy.Duration(0, JOY_TIMER_PERIOD), timer_callback, oneshot=True)
    
def callback(data):
	global key
	global var1
	# Set velocity and angle to 0.0 unless the joystick is present.
	velocity = 0.0
	angle = 0.0
	if key == 'w':
		velocity = data.velocity
		angle = data.angle
	if key == 'q':
		velocity = 0
		angle = 0

	drive_msg = AckermannDriveStamped()
	drive_msg.header.stamp = rospy.Time.now()
	drive_msg.header.frame_id = "base_link"
	drive_msg.drive.steering_angle = angle
	drive_msg.drive.speed = velocity
	pub.publish(drive_msg)
    

if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.Subscriber("drive_parameters", drive_param, callback, queue_size=1)
	joy_timer.run()
	rospy.spin()