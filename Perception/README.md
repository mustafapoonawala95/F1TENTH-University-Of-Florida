# Perception

Currently consists the implementation of the following algorithms.

--> YOLOv1 <br/>

## Introduction

YOLO, stands for You Only Look Once, is an algorithm where objects are detected in a single forward pass, hence it is a single stage detector. YOLO basically splits up an image into different cells and each of the cell outputs a set amount of bounding boxes. Those bounding boxes are responsible for detecting the objects present in a particular cell. Each bounding box in a cell consists of few elements. They are probability that an object exists in that box, coordinates and dimensions of the bounding box (like midpoint, width and height or two corners of the bounding box). <br/>

## Architecture

The network architecture is inspired by the GoogLeNet model for image classification. Our network has 24 convolutional layers followed by 2 fully connected layers. Instead of the inception modules used by GoogLeNet, we simply use 1 × 1 reduction layers followed by 3 × 3 convolutional layers. Our final layer predicts both class probabilities and bounding box coordinates. We normalize the bounding box width and height by the image width and height so that they fall between 0 and 1. We parametrize the bounding box x and y coordinates to be offsets of a particular grid cell location so they are also bounded between 0 and 1. We use a linear activation function for the final layer and all other layers use the following leaky rectified linear activation.

The architecture is shown below.
<p align = "center">
<img src="images/yolo_architecture.JPG" width="953" height="410">
</p>

# Training 

Currently the model is being trained on PASCAL VOC YOLO dataset on kaggle. mAP close to 70% was being achieved after training for 45 epochs using loss function Adam. Results will be added soon.
