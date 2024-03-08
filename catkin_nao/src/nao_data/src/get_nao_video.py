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
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import actionlib 

from naoqi import ALProxy
from naoqi_node import NaoqiNode

# NAOqi specific
import motion

armName     = "LArm"
cpt = 0
max_Pitch = 2.080
max_Roll = 1.32
min_Roll = -0.31
roll = pitch = yaw = linear_x= linear_y =linear_z = angular_x = angular_y = angular_z = 0.0
pi=3.14


class NaoqiJointStates(NaoqiNode):


	def __init__(self):
            NaoqiNode.__init__(self, 'get_nao_data')
	    #rospy.init_node('scan_values')
	    print("Connect Naoqi")
            self.connectNaoQi()
	    print("Connection done!!")
	    #default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
            self.sensorRate = rospy.Rate(rospy.get_param('~sensor_rate', 4.0))
	    tf_prefix_param_name = rospy.search_param('tf_prefix')
            if tf_prefix_param_name:
            	self.tf_prefix = rospy.get_param(tf_prefix_param_name)
            else:
            	self.tf_prefix = ""

            self.base_frameID = rospy.get_param('~base_frame_id', "base_link")
            if not(self.base_frameID[0] == '/'):
            	self.base_frameID = self.tf_prefix + '/' + self.base_frameID

            # use sensor values or commanded (open-loop) values for joint angles
            self.useJointSensors = rospy.get_param('~use_joint_sensors', True) # (set to False in simulation!)
            self.useOdometry = rospy.get_param('~use_odometry', True)
            # init. messages:
            self.torsoOdom = Odometry()
            self.torsoOdom.header.frame_id = rospy.get_param('~odom_frame_id', "odom")
            if not(self.torsoOdom.header.frame_id[0] == '/'):
            	self.torsoOdom.header.frame_id = self.tf_prefix + '/' + self.torsoOdom.header.frame_id

	    self.useJointSensors = rospy.get_param('~use_joint_sensors', True)
	    self.jointState = JointState()
            self.jointState.name = self.motionProxy.getJointNames('HeadYaw')

	    # simluated model misses some joints, we need to fill:
            if(len(self.jointState.name) == 22):
            	self.jointState.name.insert(6,"LWristYaw")
            	self.jointState.name.insert(7,"LHand")
            	self.jointState.name.append("RWristYaw")
            	self.jointState.name.append("RHand")

            msg = "Nao joints found: "+ str(self.jointState.name)
            rospy.logdebug(msg)

	    self.initRobotPosition()
	    self.useSensors = True
	    self.jointStatePub = rospy.Publisher("joint_states", JointState, queue_size=1)
	    self.cpt = 0
	    self.max_Pitch = 2.080
	    self.max_Roll = 1.32
	    self.min_Roll = -0.31
	    rospy.loginfo("nao_joint_states initialized")
	    #self.runz()


	# (re-) connect to NaoQI:
	def connectNaoQi(self):
	    rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
            self.motionProxy = self.get_proxy("ALMotion")
            self.memProxy = self.get_proxy("ALMemory")
	    self.video_service = self.get_proxy("ALVideoDevice")
	    #self.postureProxy = self.get_proxy("ALRobotPosture")
	    #self.audioProxy = self.get_proxy("ALAudioDevice")
	    #self.prefmanagerProxy = self.get_proxy("ALPreferenceManager")
            if self.motionProxy is None or self.memProxy is None:
            	exit(1)


	def initRobotPosition(self):
    	    ''' Inits NAO's position and stiffnesses to make the guiding possible.'''
	    print("MOTIIIION ROBBBOOOOOOTS!!")
    	    #self.motionProxy.wakeUp()
    	    # self.postureProxy.goToPosture("StandInit", 0.3)
	    #self.postureProxy.goToPosture("Crouch", 0.5)
    	    #self.motionProxy.moveInit()
    	    time.sleep(1.0)
	    #self.audioProxy.setOutputVolume(45)
    	    # Make left arm loose.
    	    #self.motionProxy.setAngles("LShoulderRoll", 0.0, 1.0)
	    #self.motionProxy.setAngles("LShoulderPitch", 0.0, 1.0)
    	    #self.motionProxy.setAngles("Head", [0.44, -0.44], 0.5)
    	    #self.motionProxy.setStiffnesses(armName, 0.0)
	    self.motionProxy.setStiffnesses("Head", 1.0)
	    self.motionProxy.setStiffnesses("LArm", 0.0)
    	    self.motionProxy.setStiffnesses("RArm", 0.0)
	    self.motionProxy.setStiffnesses("LLeg", 0.0)
	    self.motionProxy.setStiffnesses("RLeg", 0.0)
	    #self.prefmanagerProxy.
    	    # Disable arm moves while walking on left arm.
    	    #self.motionProxy.setMoveArmsEnabled(False, True)
    	    time.sleep(1.0)


	


	def run(self):
	    """ Odometry thread code - collects and sends out odometry esimate. """
	    resolution = 2
    	    print("resolution: {}".format(resolution))
    	    colorSpace = vision_definitions.kBGRColorSpace
    	    fps = 4
     	    time_sleep = 1/fps

   	    #nameId = video_service.subscribe("python_GVM", resolution, colorSpace, fps)
    	    nameId = self.video_service.subscribe("test1", resolution, colorSpace, fps)
    
	    #images = np.zeros([640,480,3],dtype=np.uint8)
	    if resolution == 1:
		width = 320
	    	height = 240

	    if resolution == 2:
		width = 640
	    	height = 480

	    if resolution == 3:
		width = 1280
	    	height = 960

	    image = np.zeros((height, width, 3), np.uint8)
    	    memValue = "IMGD"
	    tps = list()
	    directory = r'/home/begh/NAO_IMAGE_OUTPUT1/'
	    i=0	 	    
	    while self.is_looping():
                #
            	# Build odometry:
            	#
		tic = timeit.default_timer()		
            	try:
			#print "getting image " + str(i)
			name = directory+"img_"+ str(i)+".png"
			start_time = time.time()
                	result = self.video_service.getImageRemote(nameId);
			if result == None:
				print 'cannot capture.'
				print("--- %s seconds ---" % (time.time() - start_time))	
				#time.sleep(time_sleep)
		    	elif result[6] == None:
				print 'no image data string.'
				print("--- %s seconds ---" % (time.time() - start_time))	
				#time.sleep(time_sleep)
		    	else:
				values = result[6]
				image = np.fromstring(values, np.uint8).reshape( height, width,3 )
				#print("--- %s seconds ---" % (time.time() - start_time))
				#print("IMAGE")
				#time.sleep(time_sleep)
				cv.imwrite(name, image)
			i=i+1

            	except RuntimeError, e:
                	print "Error accessing ALMemory, exiting...\n"
                	print e
                	rospy.signal_shutdown("No NaoQI available anymore")
	    	self.sensorRate.sleep()
		toc = timeit.default_timer()
		t = abs(tic-toc)
		tps.append(t)
		
            print("OUT LOOP")
    	    self.video_service.unsubscribe(nameId)

	    output_time = os.path.abspath("/home/begh/DATA/DATA_video_time1.csv")

	    print("Writing!!")
	    with open(output_time,"w") as fpt:
		fpt.write("; \n ".join(str(y) for y in tps))

	    print("Results written to", output_time)







if __name__ == '__main__':
    
    
    joint_states = NaoqiJointStates()
    joint_states.start()

    rospy.spin()
    exit(0)




