#! /usr/bin/env python


import argparse
import time
import math
import timeit
import sys
import os
import message_filters
from datetime import datetime

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from nao_data.msg import nao_msg
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



rospy.init_node('nao_data_get')          
print("Begin synchronization!!")
#default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
sensorRate = rospy.Rate(5.0)

#NAO: data types
angle_x = 0
angle_y = 0
angle_z = 0
gyroscope_x = 0
gyroscope_y = 0
gyroscope_z = 0
acceleration_x = 0
acceleration_y = 0
acceleration_z = 0
H_yaw = 0
H_pitch = 0
LS_pitch = 0
LS_roll = 0
RS_pitch = 0
RS_roll = 0
LE_yaw = 0
LE_roll = 0
RE_yaw = 0
RE_roll = 0
LH_roll = 0
LH_pitch = 0
LH_yawpitch = 0
RH_roll = 0
RH_pitch = 0

#XSENS: data types
rate = rospy.Rate(120)
time_stamp = 0
roll = 0 
pitch = 0 
yaw = 0 
linear_x = 0 
linear_y = 0 
linear_z = 0 
angular_x = 0 
angular_y = 0
angular_z = 0

data_xsens=list()
xsens =list()
xsens.append("timestamp")
xsens.append("Roll_x")
xsens.append("Pitch_y")
xsens.append("Yaw_z")
xsens.append("omega_x")
xsens.append("omega_y")
xsens.append("omega_z")
xsens.append("alpha_x")
xsens.append("alpha_y")
xsens.append("alpha_z")
data_xsens.append(xsens)

# Output files 
output = os.path.abspath("/home/begh/DATA/DATA_test.csv")
output_time = os.path.abspath("/home/begh/DATA/DATA_test_time.csv")
output_xsens = os.path.abspath("/home/begh/DATA/imu_test.csv")

rospy.loginfo("Synchronization initialized")



def nao_callback(self,msg):
    line = list()
    time_stamp = msg.header.stamp.sec
    angle_x = msg.angle_x
    angle_y = msg.angle_y
    angle_z = msg.angle_z
    gyroscope_x = msg.gyroscope_x
    gyroscope_y = msg.gyroscope_y
    gyroscope_z = msg.gyroscope_z
    acceleration_x = msg.acceleration_x
    acceleration_y = msg.acceleration_y
    acceleration_z = msg.acceleration_z
    H_yaw = msg.H_yaw
    H_pitch = msg.H_pitch
    LS_pitch = msg.LS_pitch
    LS_roll = msg.LS_roll
    RS_pitch = msg.RS_pitch
    RS_roll = msg.RS_roll
    LE_yaw = msg.LE_yaw
    LE_roll = msg.LE_roll
    RE_yaw = msg.RE_yaw
    RE_roll = msg.RE_roll
    LH_roll = msg.LH_roll
    LH_pitch = msg.LH_pitch
    LH_yawpitch = msg.LH_yawpitch
    RH_roll = msg.RH_roll
    RH_pitch = msg.RH_pitch
    memData = [time_stamp, angle_x, angle_y, angle_z, gyroscope_x, gyroscope_y, gyroscope_z, acceleration_x, acceleration_y, acceleration_z, H_yaw, H_pitch, LS_pitch, LS_roll, RS_pitch, RS_roll, LE_yaw, LE_roll, RE_yaw, RE_roll, LH_roll, LH_pitch, LH_yawpitch, RH_roll, RH_pitch]
    for i in range(len(memData)):
	line.append(memData[i])
    
    data.append(line)
    sensorRate.sleep()
    
      

def callback(self,msg):
    xsens = list()
    #global roll, pitch, yaw, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z 
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]		    
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
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
    time_stamp = int(c)
    #time_stamp = msg.header.seq
    linear_x = msg.linear_acceleration.x
    linear_y = msg.linear_acceleration.y 
    linear_z = msg.linear_acceleration.z
    angular_x = msg.angular_velocity.x 
    angular_y = msg.angular_velocity.y 
    angular_z = msg.angular_velocity.z
    memDatax = [time_stamp,roll, pitch,yaw,angular_x ,angular_y ,angular_z ,linear_x, linear_y,linear_z]
    for i in range(len(memDatax)):
	xsens.append(memDatax[i])
    
    data_xsens.append(xsens)
    rate.sleep()


def gotdata():
    print("got data")


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

#nao_sub = rospy.Subscriber('/pub_joint_states', nao_msg, nao_callback)
#sub = rospy.Subscriber('/imu_00B42E06', Imu, callback)	
nao_datas =message_filters.Subscriber('/pub_joint_states', nao_msg)    
xsens_datas =message_filters.Subscriber('/imu_00B42E06', Imu)    

while not rospy.is_shutdown():
    ats = message_filters.ApproximateTimeSynchronizer([nao_datas, xsens_datas],queue_size=5,slop=0.1)
    ats.registerCallback(gotdata)


with open(output,"w") as fp:
    for line in data:
	fp.write("; ".join(str(x) for x in line))
	fp.write("\n")

with open(output_time,"w") as fpt:
    fpt.write("; \n ".join(str(y) for y in tps))

with open(output_xsens,"w") as fps:
    for xsens in data_xsens:
	fps.write("; ".join(str(z) for z in xsens))
	fps.write("\n")

print("Results written to", output)









