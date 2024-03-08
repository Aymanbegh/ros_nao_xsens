#! /usr/bin/env python


import argparse
import time
import math
import timeit
import sys
import os
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
#roll = pitch = yaw = linear_x= linear_y =linear_z = angular_x = angular_y = angular_z = 0.0
pi=3.14


class NaoqiJointStates(NaoqiNode):


	def __init__(self):
            NaoqiNode.__init__(self, 'get_nao_data')
	    #rospy.init_node('scan_values')
	    print("Connect Naoqi")
            self.connectNaoQi()
	    print("Connection done!!")
	    #default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
            self.sensorRate = rospy.Rate(rospy.get_param('~sensor_rate', 25.0))

	    #XSENS: data types
	    self.rate = rospy.Rate(120)
	    self.time_stamp = 0
	    self.roll = 0 
	    self.pitch = 0 
	    self.yaw = 0 
	    self.linear_x = 0 
	    self.linear_y = 0 
	    self.linear_z = 0 
	    self.angular_x = 0 
	    self.angular_y = 0
	    self.angular_z = 0

	    self.data_xsens=list()
	    self.xsens =list()
	    self.xsens.append("timestamp")
	    self.xsens.append("Roll_x")
	    self.xsens.append("Pitch_y")
	    self.xsens.append("Yaw_z")
	    self.xsens.append("omega_x")
	    self.xsens.append("omega_y")
	    self.xsens.append("omega_z")
	    self.xsens.append("alpha_x")
	    self.xsens.append("alpha_y")
	    self.xsens.append("alpha_z")
	    self.data_xsens.append(self.xsens)

	    # Output files 
	    self.output = os.path.abspath("/home/begh/DATA/DATA_test.csv")
	    self.output_time = os.path.abspath("/home/begh/DATA/DATA_test_time.csv")
	    self.output_xsens = os.path.abspath("/home/begh/DATA/imu_test.csv")


	    # NAO SENSOR LIST
	    self.dataNamesList = ["Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value","Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value", "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value", "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
                                "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value",
"Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
"Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
"Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
"Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
"Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
"Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
"Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
"Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
"Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
"Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
"Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
"Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
"Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
"Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
"Device/SubDeviceList/RHipPitch/Position/Sensor/Value"]

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


	# (re-) connect to NaoQI:
	def connectNaoQi(self):
	    rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
            self.motionProxy = self.get_proxy("ALMotion")
            self.memProxy = self.get_proxy("ALMemory")
	    #self.postureProxy = self.get_proxy("ALRobotPosture")
	    #self.audioProxy = self.get_proxy("ALAudioDevice")
	    #self.prefmanagerProxy = self.get_proxy("ALPreferenceManager")
            if self.motionProxy is None or self.memProxy is None:
            	exit(1)


	def initRobotPosition(self):
    	    ''' Inits NAO's position and stiffnesses to make the guiding possible.'''
    	    #self.motionProxy.wakeUp()
    	    time.sleep(1.0)
	    self.motionProxy.setStiffnesses("Head", 0.0)
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

	    def callback(msg):
		    self.xsens = list()
		    #global roll, pitch, yaw, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z 
		    orientation_q = msg.orientation
		    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]		    
		    (self.roll,self.pitch,self.yaw) = euler_from_quaternion(orientation_list)
		    #orientation = [roll, pitch, yaw]
		    #linear_= msg.linear_acceleration
		    a = msg.header.stamp.secs
		    b = msg.header.stamp.nsecs
		    if(b<100000000 and b>=10000000):
			d = "0"+str(b)
			c = str(a) + str(d)
			#print("Modified1")
		    elif(b<10000000 and b>=1000000):
			d = "00"+str(b)
			c = str(a) + str(d)
			#print("Modified2")	
		    elif(b<1000000 and b>=100000):
			d = "000"+str(b)
			c = str(a) + str(d)
		    elif(b<100000 and b>=10000):
			d = "0000"+str(b)
			c = str(a) + str(d)
			print("Modified4")
		    else:
			c = str(a) + str(b)
		    #c = str(a) + str(b)		  		
		    self.time_stamp = int(c)
		    #self.time_stamp = msg.header.seq
		    self.linear_x = msg.linear_acceleration.x
		    self.linear_y = msg.linear_acceleration.y 
		    self.linear_z = msg.linear_acceleration.z
		    self.angular_x = msg.angular_velocity.x 
		    self.angular_y = msg.angular_velocity.y 
		    self.angular_z = msg.angular_velocity.z
		    memDatax = [self.time_stamp,self.roll, self.pitch,self.yaw,self.angular_x ,self.angular_y ,self.angular_z ,self.linear_x, self.linear_y,self.linear_z]
		    for i in range(len(memDatax)):
			self.xsens.append(memDatax[i])
		    
		    self.data_xsens.append(self.xsens)
		    self.rate.sleep()


	    print("Run acquisition !")
	    data=list()
	    tps=list()
	    line =list()
	    line.append("Sequence")
	    line.append("angles_x")
	    line.append("angles_y")
	    line.append("angles_z")
	    line.append("angul_x")
	    line.append("angul_y")
	    line.append("angul_z")
	    line.append("acc_x")
	    line.append("acc_y")
	    line.append("acc_z")
	    line.append("head yaw")
	    line.append("head pitch")
	    line.append("Lshoulder_Pitch")
	    line.append("Lshoulder_Roll")
	    line.append("Rshoulder_Pitch")
	    line.append("Rshoulder_Roll")
	    line.append("LElbowYaw")
	    line.append("LElbowRoll")
	    line.append("RElbowYaw")
	    line.append("RElbowRoll")
	    #line.append("LWristYaw")
	    #line.append("RWristYaw")
	    line.append("LHipRoll")
	    line.append("LHipPitch")
	    line.append("LHipYawPitch")
	    line.append("RHipRoll")
	    line.append("RHipPitch")
	    data.append(line)
	    sub = rospy.Subscriber('/imu_00B42E06', Imu, callback)	    
	    while self.is_looping():
                #
            	# Build odometry:
            	#
		tic = timeit.default_timer()
            	timestamp = rospy.Time.now()
            	try:
                	memData = self.memProxy.getListData(self.dataNamesList)
			memData = [self.time_stamp, memData]
            	except RuntimeError, e:
                	print "Error accessing ALMemory, exiting...\n"
                	print e
                	rospy.signal_shutdown("No NaoQI available anymore")

		line =list()
		self.xsens= list()

		for i in range(len(memData)):
			line.append(memData[i])

		data.append(line)
	    	self.sensorRate.sleep()
		toc = timeit.default_timer()
		t = abs(tic-toc)
		tps.append(t)


	    with open(self.output,"w") as fp:
		for line in data:
			fp.write("; ".join(str(x) for x in line))
			fp.write("\n")

	    with open(self.output_time,"w") as fpt:
		fpt.write("; \n ".join(str(y) for y in tps))

	    with open(self.output_xsens,"w") as fps:
		for self.xsens in self.data_xsens:
			fps.write("; ".join(str(z) for z in self.xsens))
			fps.write("\n")

	    print("Results written to", output)





if __name__ == '__main__':
    
    
    joint_states = NaoqiJointStates()
    joint_states.start()

    rospy.spin()
    exit(0)




