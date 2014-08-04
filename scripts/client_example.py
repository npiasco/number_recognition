#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from number_recognition.srv import *

def read_floor_number_client(im):
    rospy.wait_for_service('read_floor_number')
    try:
        read_floor_number = rospy.ServiceProxy('read_floor_number', ReadFloorNumber)
        resp = read_floor_number(im)
        print "floor :  %s"%(resp.floorNumber)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":

    rospy.init_node('client', anonymous=True)
    print "Requesting srv"
    image_sub = rospy.Subscriber("head_xtion/rgb/image_color",Image,read_floor_number_client)
    rospy.spin()
