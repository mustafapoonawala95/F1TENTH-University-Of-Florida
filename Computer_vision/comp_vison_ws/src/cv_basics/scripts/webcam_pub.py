#!/usr/bin/env python
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import Header
import numpy as np
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  #pub = rospy.Publisher('video_frames', Image, queue_size=10)
  image_pubulish=rospy.Publisher('/camera/image_raw',Image,queue_size=1)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
  rate = rospy.Rate(10)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(2) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(0)
     
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame (%dx%d)',cap.get(3),cap.get(4))
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        #pub.publish(br.cv2_to_imgmsg(frame))

	image_temp=Image()
        header = Header(stamp=rospy.Time.now())
    	header.frame_id = 'map'
    	#image_temp.height=IMAGE_HEIGHT
    	#image_temp.width=IMAGE_WIDTH
    	image_temp.encoding='rgb8'
    	image_temp.data=np.array(cv2.imencode('.jpg', frame)[1]).tostring()
    	#print(imgdata)
    	#image_temp.is_bigendian=True
    	image_temp.header=header
    	#image_temp.step=1241*3
    	image_pubulish.publish(image_temp)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
