#!/usr/bin/env python
"""OpenCV feature detectors with ros Image Topics in python.

This example subscribes to a ros topic containing sensor_msgs
Image. It converts the Image into a numpy.ndarray,
then detects and marks features in that image. It finally displays
and publishes the new image - again as Image topic.
"""
__author__ =  'Thor Jensen'
__version__=  '0.1'

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/waypoints",
            Image,queue_size = 1)
        self.image_pub = rospy.Subscriber("/output/image_treated",
            Image,queue_size = 1)
        #self.image_pub_display = rospy.Publisher("/display_image",Image,queue_size =1)
        # self.bridge = CvBridge()

        # subscribed Topic
	#self.subMark7 = rospy.Subscriber("/markerlocater/markerpose7",
     #       Image, self.callback,  queue_size = 1)
	#self.subMark8 = rospy.Subscriber("//markerlocater/markerpose7",
      #      Image, self.callback,  queue_size = 1)
       # self.subscriber = rospy.Subscriber("/image_cropped",
        #    Image, self.callback,  queue_size = 1)
        #if VERBOSE :
         #   print "subscribed to /camera/image/compressed"



def main(args):
    '''Initializes and cleanup ros node'''
    image_feature()
    rospy.init_node('astar', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

