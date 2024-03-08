#! /usr/bin/env python


import argparse
import time
import math
import timeit
import sys
import os
import vision_definitions
import cv2 as cv
import numpy as np
import base64

from datetime import datetime

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import actionlib 

from naoqi import ALProxy
from naoqi_node import NaoqiNode

# NAOqi specific
import motion




def callback(self):
    #Write image
    image = np.zeros((self.height, self.width, 3), np.uint8)		
    name = self.directory+ str(self.msg.header)+".png"       
    values = self.img.data
    image = np.fromstring(values, np.uint8).reshape( self.height, self.width,3 )
    cv.imwrite(name, image)




def main(self):
    """ Odometry thread code - collects and sends out odometry esimate. """

    #images = np.zeros([640,480,3],dtype=np.uint8)
    resolution = 2
    if resolution == 1:
	self.width = 320
    	self.height = 240

    if resolution == 2:
	self.width = 640
    	self.height = 480

    if resolution == 3:
	self.width = 1280
    	self.height = 960

    self.img = Image()
    self.imageSub = rospy.Subscriber("image_nao",Image,self.callback);
    self.directory = r'/home/begh/NAO_IMAGE_OUTPUT1/'
    while not rospy.is_shutdown():
	print("IMAGE")
	   

    print("Results written to")


	


	






if __name__ == '__main__':
    
    
    main()
    rospy.spin()
    exit(0)




