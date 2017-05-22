# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String

com = cv2.imread("compare.png")        #this i cheating. the cropped image

def callback(data):
    rospy.loginfo(rospy.get_caller_id() )
    fp=("path.txt","w")
    fp.write(data.data)
    fp.close
    img = np.loadtxt("path.txt",skiprows=4)
    save(img)
    newImg=cv2.imread("tmp.png")
    p = path(newImg)
    merge(p,com)

def path(image):
    image[image ==5]=255        #setting path to white
    image[image !=255]=1        #Black everything else
    return image

def merge(image,image2):
    return image * image2

def save(image):
    cv2.imwrite('tmp.png',image)



def listener():
    rospy.init_node('draw_route',anonymous=False)
    rospy.Subscriber('path',String,callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
