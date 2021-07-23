#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 15 23:56:37 2021

@author: krishna
"""
import tensorflow as tf # Machine learning library
import numpy as np # Scientific computing library
import cv2 # Computer vision library


# Model URL
# model_name = "ssd_mobilenet_v2_fpnlite_640x640_coco17_tpu-8"
model_name = "ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8"
url = 'http://download.tensorflow.org/models/object_detection/tf2/20200711/' + model_name + '.tar.gz'

#Load the model
model_dir = tf.keras.utils.get_file(fname=model_name, untar=True, origin=url)
print("Model path: ", str(model_dir))
model_dir = str(model_dir) + "/saved_model"
model = tf.saved_model.load(str(model_dir))

def getBox(output, count, shape):
	box        = np.array(output['detection_boxes'][0][count])
	startPOint = (int(box[1]*shape[1]), int(box[0]*shape[0]))
	endPoint   = (int(box[3]*shape[1]), int(box[2]*shape[0]))
	return startPOint, endPoint

# Color in RGB
color = (255, 0, 0)
# Line thickness
thickness = 2

def perform_obj_detection(current_frame):
	input_tensor = tf.convert_to_tensor(current_frame) # Input needs to be a tensor
	input_tensor = input_tensor[tf.newaxis, ...]

	# Run the model
	output = model(input_tensor)

	count = 0

	for detection_score in output['detection_scores'][0]:
        
		if float(detection_score) < 0.32:
			break
        
		if int(output['detection_classes'][0][count]) == 10:
			(startPoint, endPoint) = getBox(output, count, current_frame.shape)
			current_frame = cv2.rectangle(current_frame, startPoint, endPoint, color, thickness)
        
		count += 1

	return current_frame

        
        



