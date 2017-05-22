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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#app import
from feature_detection.msg import coord
from markerlocator.msg import markerpose

# Globals
VERBOSE=False
timestamp=[]
x=[]            #List of x coord from corners
y=[]            #List of y coord from corners
bridge=CvBridge()
mark4Flag = False
mark5Flag = False
mark6Flag = False
mark8Flag = False
targetFlag = False
frobitFlag = False
fx = 0            #frobits x coord
fy = 0            #frobits y coord
tx = 0            #targets x coord
ty = 0            #targets y coord
theta = 0         #orientation of the frobit
Qthresh = 0.007    # threshold for the quality
color_img= np.ndarray


def markerCrop(xList,yList,image):
    hx = int(max(xList))
    lx = int(min(xList))
    hy = int(max(yList))
    ly = int(min(yList))
    image[ly:hy,lx:hx]
    return cropped


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



def row(image):                                   #Prints the # of rows
    rows=0
    for col in image:
        for row in col:
            rows +=1
    return rows/coloum(image)

def coloum(image):                                #Prints the # of coloums
    colum = 0
    for col in image:
        colum+=1
    return colum


def makeMatrix(image,tx,ty):                            #Makes a .txt file acordingly to the threshold values
    tmp = image
    #tmp[tmp ==255]=7
        #print "im in elif"
    #cv2.imshow('masking',image)
    np.savetxt('matrix.txt',image,fmt='%1d',header='%i\n%i\n%i\n%i' %(row(image),coloum(image),tx,ty),comments='')##missing robot pos 50 ,50
    #np.savetxt('colorScale.txt',image,fmt='%1d',header='%i\n%i\n%i\n%i' %(row(image),coloum(image),50,50),comments='')


def findRed(image):                               #Finds Red in image
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_limit=np.array([0,100,140])             #threshold
    upper_limit=np.array([20,255,255])            #threshold
    mask = cv2.inRange(hsv,lower_limit,upper_limit)#masking
    #res=cv2.bitwise_and(image,image,mask=mask)     #result after masking
    #outI(res)
    #outI(mask)
    return mask

def colorScale(image):
    image[image == 255] = 7

def colorScale2(image):
    image[image == 255] = 150


def save(image):
    cv2.imwrite('feature_de_RGB_crop.png',image)


class image_feature:

    def __init__(self):

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_treated",
            String,queue_size = 1)

        self.image_pub_display = rospy.Publisher("/display_image",Image,queue_size =1)

        self.coord_pub = rospy.Publisher("/output/target_frobit",coord,queue_size = 1)

        # subscribed Topic
        self.raw_sub = rospy.Subscriber("/markercapture/raw",Image, self.callbackMaster,queue_size = 1)

        self.subMark4 = rospy.Subscriber("/markerlocator/markerpose_4",
            markerpose, self.callback4,  queue_size = 1) # UpperLeft Corner
        self.subMark5 = rospy.Subscriber("/markerlocator/markerpose_5",
            markerpose, self.callback5,  queue_size = 1) # LowerRight Corner

        self.subMark6 = rospy.Subscriber("/markerlocator/markerpose_6",
            markerpose, self.callback6,  queue_size = 1) # UpperRight Corner

        self.subMark7 = rospy.Subscriber("/markerlocator/markerpose_7",
            markerpose, self.frobit,  queue_size = 1) # The Frobit

        self.subMark8 = rospy.Subscriber("/markerlocator/markerpose_8",
            markerpose, self.callback8,  queue_size = 1) # LowerLeft Corner

        self.subMark9 = rospy.Subscriber("/markerlocator/markerpose_9",
            markerpose, self.target,  queue_size = 1) # Target

        self.subscriber = rospy.Subscriber("/image_cropped",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

    def callback4(self,ros_data):
        global mark4Flag
        if ros_data.quality >Qthresh :
            #print "flag 4 - good quality"
            mark4Flag = True
            x.append(ros_data.x)
            y.append(ros_data.y)

    def callback5(self,ros_data):
        global mark5Flag
        if ros_data.quality >Qthresh :
            mark5Flag = True
            x.append(ros_data.x)
            y.append(ros_data.y)


    def callback6(self,ros_data):
        global mark6Flag
        if ros_data.quality >Qthresh :
            mark6Flag = True
            x.append(ros_data.x)
            y.append(ros_data.y)

    def callback8(self,ros_data):
        global mark8Flag
        if ros_data.quality >Qthresh :
            mark8Flag = True
            #print mark8Flag
            x.append(ros_data.x)
            y.append(ros_data.y)

    def target(self,ros_data):
        global targetFlag
        if ros_data.quality >Qthresh :
            targetFlag = True
            global tx
            tx = ros_data.x
            global ty
            ty = ros_data.y

    def frobit(self,ros_data):
        global frobitFlag
        if ros_data.quality >Qthresh :
            frobitFlag = True
            global fx
            fx= ros_data.x
            global fy
            fy= ros_data.y
            global theta
            theta = ros_data.theta

    def callbackMaster(self,ros_data):
        img=bridge.imgmsg_to_cv2(ros_data,"bgr8")
        print "entered master"
        global mark4Flag
        global mark8Flag
        global mark6Flag
        global mark5Flag
        print mark4Flag, mark8Flag, mark6Flag,mark5Flag
        if  mark4Flag and mark5Flag and mark6Flag and mark8Flag:
            print "flags match"
            hx = int(max(x))
            lx = int(min(x))
            hy = int(max(y))
            ly = int(min(y))
            new_img = img[ly:hy, lx:hx]
            save(new_img)
            cv2.imshow("cropped",new_img) # change to new_img for crop
            cv2.waitKey(5)
            cv2.imwrite('the_raw_image.png',img)
            obj = findRed(new_img)
            cv2.imwrite('red_found.png',obj)
            colorScale(obj)
            #sc=colorScale(obj)
            cv2.imwrite('color_scale.png',obj)
            if VERBOSE:
                cv2.imshow('the raw image',img)
                cv2.imshow('consist only of 0 and 7',obj)
                cv2.waitKey(2)
            makeMatrix(obj)
            if not rospy.is_shutdown():
                file_obj= open("matrix.txt","r")        #Read-only matrix.txt
                r = file_obj.read()
            self.image_pub.publish(r)
#            msg=bridge.cv2_to_imgmsg(obj, encoding="8UC1")
            #self.image_pub.publish(msg)
            cmsg = coord()
            cmsg.fx = fx - lx
            cmsg.fy = fy - ly
            cmsg.theta = theta
            cmsg.tx = tx - lx
            cmsg.ty = ty - ly
            self.coord_pub.publish(cmsg)
            mark4Flag = False
            mark8Flag = False
            mark6Flag = False
            mark5Flag = False


    def callback(self,ros_data):
        img = bridge.imgmsg_to_cv2(ros_data,"bgr8")
        obj=findRed(img)
        colorScale(obj)
        #makeMatrix(obj)
        #save(obj)
        if VERBOSE:
            cv2.imshow('the raw image',img)
            cv2.imshow('consist only of 0 and 7',obj)
            cv2.waitKey(2)
        msg=bridge.cv2_to_imgmsg(obj, encoding="8UC1")
        msg2=bridge.cv2_to_imgmsg(img,encoding="bgr8")
        self.image_pub.publish(msg)
        self.image_pub_display.publish(msg2)
        #self.subscriber.unregister()



def main(args):
    '''Initializes and cleanup ros node'''
    image_feature()
    rospy.init_node('feature_detect', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

