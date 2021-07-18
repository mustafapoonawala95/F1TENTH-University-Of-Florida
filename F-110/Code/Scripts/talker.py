#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np

def talker():
    topic = 'floats'
    pub = rospy.Publisher(topic, numpy_msg(Floats))
    rospy.init_node('talker_node', anonymous=True)
    r = rospy.Rate(10) # 10hz
    rospy.loginfo("I will publish to the topic %s", topic)
    while not rospy.is_shutdown():
        a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        pub.publish(a)
        r.sleep()
 
if __name__ == '__main__':
    talker()
