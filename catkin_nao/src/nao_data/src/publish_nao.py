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
from std_msgs.msg import String
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from nao_data.msg import nao_msgs
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
            NaoqiNode.__init__(self, 'nao_data')
	    #rospy.init_node('scan_values')
	    print("Connect Naoqi")
            self.connectNaoQi()
	    print("Connection done!!")
	    #default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
            self.sensorRate = rospy.Rate(rospy.get_param('~sensor_rate', 50.0))



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

	    self.useSensors = True
	    #self.jointStatePub = rospy.Publisher("joint_states", JointState, queue_size=5)
	    print("Publishing data")
	    self.nao_publisher()


	# (re-) connect to NaoQI:
	def connectNaoQi(self):
	    rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
            self.motionProxy = self.get_proxy("ALMotion")
            self.memProxy = self.get_proxy("ALMemory")
	    self.motionProxy.setStiffnesses("Head", 1.0)
	    #self.postureProxy = self.get_proxy("ALRobotPosture")
	    #self.audioProxy = self.get_proxy("ALAudioDevice")
	    #self.prefmanagerProxy = self.get_proxy("ALPreferenceManager")
            if self.motionProxy is None or self.memProxy is None:
            	exit(1)


	def nao_publisher(self):
	    #self.rospy.init_node('nao_data', anonymous=True)
	    self.jointStatePub = rospy.Publisher("pub_nao_states", nao_msgs, queue_size=10)	  
	    while not rospy.is_shutdown():
		memData = self.memProxy.getListData(self.dataNamesList) 
		data = nao_msgs()
		t = rospy.Time.from_sec(time.time())
                #print("Time : {}".format(t))
   		#seconds = t.to_sec() #floating point
   		nanoseconds = t.to_nsec()
		data.header.stamp.secs = int(str(nanoseconds)[0:-9])
		data.header.stamp.nsecs = int(str(nanoseconds)[-9:])
		#print("Time : {}   and {}   and {}".format(nanoseconds, t, seconds))
		data.angle_x = memData[0]
		data.angle_y = memData[1]
		data.angle_z = memData[2]
		data.gyroscope_x = memData[3]
		data.gyroscope_y = memData[4]
		data.gyroscope_z = memData[5]
		data.acceleration_x = memData[6]
		data.acceleration_y = memData[7]
		data.acceleration_z = memData[8]
		data.H_yaw = memData[9]
		data.H_pitch = memData[10]
		data.LS_pitch = memData[11]
		data.LS_roll = memData[12]
		data.RS_pitch = memData[13]
		data.RS_roll = memData[14]
		data.LE_yaw = memData[15]
		data.LE_roll = memData[16]
		data.RE_yaw = memData[17]
		data.RE_roll = memData[18]
		data.LH_roll = memData[19]
		data.LH_pitch = memData[20]
		data.LH_yawpitch = memData[21]
		data.RH_roll = memData[22]
		data.RH_pitch = memData[23]
		self.jointStatePub.publish(data)
		self.sensorRate.sleep()



if __name__ == '__main__':
    
    
    joint_states = NaoqiJointStates()
    joint_states.start()

    rospy.spin()
    exit(0)




