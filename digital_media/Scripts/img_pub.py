# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2

from cv_bridge import CvBridge


img = cv2.imread("GreenMiddle.png")
def autoCrop(image):                              #Cut the usable workspace from image
    xlist=[]
    ylist=[]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_limit=np.array([60,100,140])             #threshold
    upper_limit=np.array([100,255,255])            #threshold
    mask = cv2.inRange(hsv,lower_limit,upper_limit)#masking
    res=cv2.bitwise_and(image,image,mask=mask)     #result after masking
    #cv2.imwrite('maskedCorners.png',res)           #saving the picture
    #print "hsv values UpperLeft", hsv[126][400]
    #print "hsv values UpperRight", hsv[100][880]
    #print "hsv values LowerLeft", hsv[393][393]
    #print "hsv values LowerRight", hsv[412][904]
    tmp = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)      #coverting resulting into color-scale
    bwimg = cv2.cvtColor(tmp,cv2.COLOR_BGR2GRAY)   #converting colorpic into GREY-scale
    #cv2.imwrite('greyCorn.png',bwimg)              #output greyscale of matching color found
#Finding coherent elements
    if cv2.__version__[0]=='3': #For opencv 3.0
       _, contours, hierarchy = cv2.findContours(bwimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else: #For opencv <3.0
        contours, hierarchy = cv2.findContours(bwimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        areal = cv2.contourArea(cnt)
        print "areal ", areal
        if(areal)>1:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])                ## ???
            cy = int(M['m01']/M['m00'])                ## ???

        if(areal) >12:
            #print "im in"
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])                ## ???
            cy = int(M['m01']/M['m00'])                ## ???

            xlist.append(cx)                    #list containing center x coord
            ylist.append(cy)                    #list containing center y coord
            #print "length", len(xlist)
            ly = min(ylist)
            hy =max(ylist)
            lx=min(xlist)
            hx=max(xlist)
    #print "hy = ", hy
    #print "xlist", xlist
    #print "lx %i , hx %i     ly %i , hy %i" %(lx,hx,ly,hy)
    crop_img = image[ly:hy,lx:hx]
    #cv2.imshow('cropped image',crop_img)
    return crop_img


def img_pub():
    bridge=CvBridge()
    rospy.init_node('img_pub',anonymous=False)
    pub= rospy.Publisher("/image_cropped",Image, queue_size=10)
    rate = rospy.Rate(1)
    image = autoCrop(img)
    while not rospy.is_shutdown():
        msg=bridge.cv2_to_imgmsg(image, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        print "Main"
        img_pub()
    except rospy.ROSInterruptException:
        pass
