#! /usr/bin/env python

import rospy
import argparse
import csv
import time
import math
import timeit
import sys
import os
from sensor_msgs.msg import Imu



pi=3.14


class SaveIMU(object):
	def __init__(self):
		self.time_stamp = 0
		self.w_x = 0
		self.w_y = 0
		self.w_z = 0
		self.a_x = 0
		self.a_y = 0
		self.a_z = 0
		self.data_xsens=list()
		self.xsens =list()
		self.xsens.append("timestamp")
		self.xsens.append("omega_x")
		self.xsens.append("omega_y")
		self.xsens.append("omega_z")
		self.xsens.append("alpha_x")
		self.xsens.append("alpha_y")
		self.xsens.append("alpha_z")
		self.data_xsens.append(self.xsens)
		self.output_xsens = os.path.abspath("/home/begh/NAO_IMU_OUTPUT2/imu0/imu0.csv")
		self.rate = rospy.Rate(120)
		#self.imu_dict ={"time":self._imu_t, "velocity":self._imu_w,"acceleration":self._imu_a}
		print("launch")		
		self._imu_sub = rospy.Subscriber('/imu_00B42E12', Imu, self.sub_callback)
		#self.rate.sleep()
		self.write_to_file()
		self.rate.sleep()

	def sub_callback(self,msg):
		#print("call")
		#self.time_stamp = msg.header.seq
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
		self.w_x = msg.angular_velocity.x
		self.w_y = msg.angular_velocity.y		
		self.w_z = msg.angular_velocity.z
		self.a_x = msg.linear_acceleration.x
		self.a_y = msg.linear_acceleration.y
		self.a_z = msg.linear_acceleration.z
		#print("--------- Listening: {}".format(self.time_stamp))
		self.rate.sleep()

	def write_to_file(self):
		while not rospy.is_shutdown():		
			memDatax = [self.time_stamp, self.w_x,self.w_y,self.w_z,self.a_x,self.a_y,self.a_z]
			self.xsens = list()
			for i in range(len(memDatax)):
				self.xsens.append(memDatax[i])
			#print("writting: {}".format(self.time_stamp))
			#print("XSENS")
			self.data_xsens.append(self.xsens)
			self.rate.sleep()


		with open(self.output_xsens,"w") as fps:
			for xsens in self.data_xsens:
				fps.write("; ".join(str(z) for z in xsens))
				fps.write("\n")

		    	print("Results written to", self.output_xsens)

			rospy.loginfo("written")


if __name__ == '__main__':
	rospy.init_node('spot_recorder', log_level =rospy.INFO)
	save_spot_object = SaveIMU()
