#!/usr/bin/env python3

import rospy
import numpy as np
from os import system
import time

import Microcontroller_Manager_Serial as Serial
import Modem_Functions as Modem
import Pressure_Functions as Pressure

from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

P0 = 1.01325 #Default Pressure 
pub_cmd = rospy.Publisher('/from_modem', Int16, queue_size=1)
pub_feed = rospy.Publisher('/feedback',Vector3,queue_size=1)

def Start_up():
	global P0
	# Open the Serial Port - Custom function - Baud Rate 921600
	Serial.Serial_Port_Standard()
	time.sleep (0.5)
	# Wake Modem from StandBy
	Modem.Send_Data(1, 99, 1, [75])
	# Get Atmosferic Pressure
	data_in = Pressure.Pressure_Get_Final_Values(1,1)
	P0 = (np.int16((data_received[6]<<24) | (data_received[7]<<16) | (data_received[8]<<8) | (data_received[9])))/10000
	
def Check_modem():
	# Check for new modem incoming
	data_in = Serial.Serial_Port_Receive_Data(10,2)
	try:
		if (data_in[0] == 91) # Received data from acoustic modem
			from_modem = data_in[8]
			pub_cmd.publish(from_modem) # Publish to topic
			rospy.loginfo(('Recieved: ', from_modem))
	except:
		rospy.loginfo('No New Modem Input')
		
def Check_sensors():
	data_received = Pressure.Pressure_Get_Final_Values(1,1)
	P1 = (np.int16((data_received[6]<<24) | (data_received[7]<<16) | (data_received[8]<<8) | (data_received[9])))/10000
	P = P1 - P0 # Relative Measured Pressure
	data_received = IMU.IMU_Get_Values(1, 4)
	Y = np.int16((data_received[6]<<8) | (data_received[7])) # Yaw Rotation Speed
	feedback =Vector3()
	feedback.x = Y      #Angular Velocity
	feedback.y = 0
	feedback.z = P/9.81 #Depth
	pub_feed.publish(feedback)
	

def Control_motors(m):
	m0 = m.x
	m1 = m.y
	m2 = m.z
	
	d0 = np.uint8(np.sign(m0))
	d1 = np.uint8(np.sign(m1))
	d2 = np.uint8(np.sign(m2))
	if d0 == -1 : d0 = 0
	if d1 == -1 : d1 = 0
	if d2 == -1 : d2 = 0
	s0 = np.uint8(abs(m0))
	s1 = np.uint8(abs(m1))
	s2 = np.uint8(abs(m2))
	
	Motor.Control_All_Motors(1,[d0,d2,d1],[s0,s2,s1])
	
	


if __name__ == '__main__':
	Start_up()
	rospy.init_node('serial_bridge')
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		
		Check_modem()
		
		try:
			Check_sensors()
			rospy.Subscriber("/motors", Vector3, Control_motors)
		except:
			print('skip')
		rate.sleep()

			
		
