# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2

from cv_bridge import CvBridge



bgr = cv2.imread("the_raw_image.png")
grey=cv2.imread("the_raw_image.png",0)
def img_pub():
    bridge=CvBridge()
    rospy.init_node('capture_spoofer',anonymous=False)
    col=rospy.Publisher("/markercapture/raw",Image,queue_size=1)

    not_col= rospy.Publisher("/markerlocator/image_raw",Image, queue_size=1)
    rate = rospy.Rate(.5)

    #image = autoCrop(img)
    #cv2.imshow("frame",image)

    #cv2.waitKey(0)
    while not rospy.is_shutdown():
        msg=bridge.cv2_to_imgmsg(bgr,encoding="bgr8")
        msg2=bridge.cv2_to_imgmsg(grey, encoding="8UC1")
        col.publish(msg)
        not_col.publish(msg2)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        print "Main"
        img_pub()
    except rospy.ROSInterruptException:
        pass
