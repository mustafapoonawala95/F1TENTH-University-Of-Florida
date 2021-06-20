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
 
#Load model
weights_path = '/home/krishna/my_first_ws/src/cv_basics/scripts/yolov3_training_last.weights'
config_path = '/home/krishna/my_first_ws/src/cv_basics/scripts/yolov3_testing.cfg'
net = cv2.dnn.readNet(weights_path, config_path)
font = cv2.FONT_HERSHEY_PLAIN
colors = np.random.uniform(0, 255, size=(100, 3))

def callback(data):
 
	# Output debugging information to the terminal
	rospy.loginfo("receiving video frame")

	# Convert ROS Image message to OpenCV image
	np_arr = np.fromstring(data.data, np.uint8)
	current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	# Display image
	#cv2.imshow("camera", current_frame)
	cv2.imshow("camera", detector(current_frame))
	cv2.waitKey(1)

def detector(img):
	classes = [""]
	height = 480
	width  = 640

	blob = cv2.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
	net.setInput(blob)
	output_layers_names = net.getUnconnectedOutLayersNames()
	layerOutputs = net.forward(output_layers_names)

	boxes = []
	confidences = []
	class_ids = []

	for output in layerOutputs:
	    for detection in output:
	        scores = detection[5:]
	        class_id = np.argmax(scores)
	        confidence = scores[class_id]
	        if confidence > 0.2:
	            center_x = int(detection[0]*width)
	            center_y = int(detection[1]*height)
	            w = int(detection[2]*width)
	            h = int(detection[3]*height)

	            x = int(center_x - w/2)
	            y = int(center_y - h/2)

	            boxes.append([x, y, w, h])
	            confidences.append((float(confidence)))
	            class_ids.append(class_id)

	indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)

	if len(indexes)>0:
	    for i in indexes.flatten():
	        x, y, w, h = boxes[i]
	        label = str(classes[class_ids[i]])
	        confidence = str(round(confidences[i],2))
	        color = colors[i]
	        cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
	        cv2.putText(img, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)
	return img
      
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
