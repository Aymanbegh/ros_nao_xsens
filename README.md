# README NAO/XSENS within ROS use

The following program is used to retrieve sensor data from the NAO robot and inertial data from the Xsens mTw IMU under ROS.

- Start the XSENS system first:

Don't forget to find the rostopic of the iMU used (rostopic list)
type into a terminal
roscore 

cd catkin_xsens

source devel/setup.bash

in another terminal
rosrun xsens_mtw_driver mt_w_manager


# 2 non-synchronised/synchronised configurations: 

**1) Via catkin_nao and the nao_data package only** 
Non-synchronised retrieval but with the corresponding sequences from each of the systems (nao/xsens):
- Allows values to be matched
- Keeps a high frequency for the IMU

use the command: roslaunch nao_data get_nao_data.launch /// or rosrun nao_data get_nao_xsens_data_nosync.py
- call the python function: get_nao_xsens_data_nosync.py
- synchronises data writing at the frequencies of each of the systems but diff recovery (xsens: 120Hz / nao: max 50 Hz)


**2) Via catkin_nao and the nao_data and nao_sync packages only**
Synchronised recovery of data from each of the systems (nao/xsens):
- Allows the sensors to be synchronised to the one with the lowest frequency (NAO: max 50Hz)

Use the command: roslaunch nao_data set_nao_data.launch /// or rosrun nao_data publish_nao.py
- call the python function: publish_nao.py
- create a publisher which sends sensor data at the chosen frequency (nao: max 50 Hz)

Then use the command: roslaunch nao_sync get_nao_sync.launch /// or rosrun nao_sync get_nao_xsens_data_sync.py
- call the python function: get_nao_xsens_data_sync.py
- Create two subscribers that retrieve synchronized data from the xsens and nao systems at the lowest frequency between the sensors (nao: max 50 Hz)


++++++++++++++++++++++++++++++++++++ Publish the data in CSV files
