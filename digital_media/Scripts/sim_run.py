# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id()  + ' I recieved %s' , data.data)
    fh=open("r_v1.txt","w")
    fh.write(data.data)
    fh.close
    program ='./a.exe'
    arg = 'r_v1.txt'
    subprocess.call([program, arg])

def listener():
    rospy.init_node('astar',anonymous=False)
    rospy.Subscriber('/output/image_treated', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
