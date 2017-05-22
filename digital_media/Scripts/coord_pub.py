#!/usr/bin/env python
# Python libs
import sys, time
import math

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#app import
from feature_detection.msg import coord
from markerlocator.msg import markerpose


#Class for sending hardcoded coords messages in order to bypass the whole system when testing code
class image_feature:

    def __init__(self):
        self.coord_pub = rospy.Publisher("/output/target_frobit",coord,queue_size = 1)
        cmsg = coord()
        cmsg.fx = 4
        cmsg.fy = 5
        cmsg.theta = 0
        cmsg.tx = 6
        cmsg.ty = 6
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print "Its something"
            self.coord_pub.publish(cmsg)
            rate.sleep()


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('coord_pub', anonymous=False)
    image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image coord pub module"


if __name__ == '__main__':
    main(sys.argv)
