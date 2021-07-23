#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
import numpy as np
import detector
 

def callback(data):
 
	# Output debugging information to the terminal
	rospy.loginfo("receiving video frame")

	# Convert ROS Image message to OpenCV image
	np_arr = np.fromstring(data.data, np.uint8)
	current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	# Display image
	#cv2.imshow("camera", current_frame)
	cv2.imshow("camera", detector.perform_obj_detection(current_frame))
	cv2.waitKey(1)  
      
def receive_message():
 
	# Tells rospy the name of the node.
	# Anonymous = True makes sure the node has a unique name. Random
	# numbers are added to the end of the name. 
	rospy.init_node('video_sub_py', anonymous=True)

	# Node is subscribing to the video_frames topic
	#rospy.Subscriber('video_frames', Image, callback)
	rospy.Subscriber('/camera/image_raw', Image, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	# Close down the video stream when done
	cv2.destroyAllWindows()
  
if __name__ == '__main__':
	receive_message()
