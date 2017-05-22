# -*- coding: utf-8 -*-
#!/usr/bin/env python

#import rospy
import cv2
import numpy as np
#from std_msgs.msg import String

com = cv2.imread('feature_de_RGB_crop.png')

def save(image):
    cv2.imwrite('tmp.png',image)    

def path(image):
    image[image == 0]=1        #so it is possible to merge images
    image[image ==4]=255        #setting start to white
    image[image == 6] = 255    #setting end to white
    image[image == 8]= 200      #Waypoints
    image[image == 5]= 120      #path
    #Black everything else
    return image

def merge(image,image2):
    return image * image2

def drawRoute():
    img = np.loadtxt("Map.txt")
    cv2.imshow('raw',img)
    save(img)
    newImg=cv2.imread('tmp.png')
    p = path(newImg)
    merg = merge(p,com)
    cv2.imshow('my paths',p)
    cv2.imshow('paths merge into start',merg)
    cv2.waitKey(0)


drawRoute()
