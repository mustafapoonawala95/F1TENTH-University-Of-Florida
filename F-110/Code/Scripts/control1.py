#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np

# TODO: modify these constants to make the car follow walls smoothly.
KP = 20.0
KD = 0.01
limit_ang = 20
prev_error = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
  # TODO: Based on the error (data.data), determine the car's required velocity
  # amd steering angle.
  	global prev_error
	global limit_ang
	velocity = 1.0
	error = 5*data.data
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
	msg.velocity = 1.5  # TODO: implement PID for velocity
	msg.angle = angle   # TODO: implement PID for steering angle
	pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()

