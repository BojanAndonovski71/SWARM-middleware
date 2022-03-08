
# ----------------------------------------------------------------------------
# BSD 3-Clause License

# Copyright (c) 2021, Gaspare Santaera, Istituto di Biorobotica
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

## @package Microcontroller_Manager_Bluetooth
#
#  This python module contains all the functions to communicate with the robotic fish throught bluetooth
#  
#  The high-level microprocessor (e.g. Raspberry PI) can communicate with the 
#  the microcontroller inside the robotic fish by the serial port or throught the bluetooth.

import bluetooth
import Microcontroller_Manager_Serial as Serial
Bluetooth_target_name_ = None
Bluetooth_target_address_ = None 
Bluetooth_target_port_ = 1
Bluetooth_sock_ = bluetooth.BluetoothSocket(bluetooth.RFCOMM)


## Function which scans and returns all the bluetooth device reachable
def Bluetooth_Device_Scan():

	global Bluetooth_target_name_
	global Bluetooth_target_address_ 
	global Bluetooth_target_port_
	global Bluetooth_sock_
	target_address_ = None
	nearby_devices = bluetooth.discover_devices(lookup_names=True)
	print()
	print("Found {} devices.".format(len(nearby_devices)))
	index = 1
	for addr, name in nearby_devices:
		print("{} - {} - {}".format(index, name, addr))
		index = index + 1
	print("Which device you want to connect to? ")
	input_var = int(input())-1
	if input_var < index-1:
		target_name = nearby_devices[input_var][1]
		target_address = nearby_devices[input_var][0]
		Bluetooth_target_name_ = target_name
		Bluetooth_target_address_ = target_address
		Bluetooth_sock_.connect((Bluetooth_target_address_, Bluetooth_target_port_))

## Function which create a connection with the chosen device
def Bluetooth_Device_Open():

	global Bluetooth_sock_
	data_to_send = [63, 13, 10]
	Bluetooth_sock_.send(bytes(data_to_send))
	Bluetooth_sock_.settimeout(2.0)
	data_read = ""
	for i in range(5):
		try:
			data_received = Bluetooth_sock_.recv(500)
			if data_received:
				data_read = data_read + data_received.decode()
		except bluetooth.btcommon.BluetoothError:
			pass
	print()
	print(data_read)

	#socket.close()

## Function which sends a selected package to the microcontroller throuth the bluetooth
def Bluetooth_Send_Data(data_to_send):

	global Bluetooth_sock_
	Bluetooth_sock_.send(bytes(data_to_send))

## Function which receives a package from the microcontroller throuth the bluetooth
def Bluetooth_Receive_Data(current_data_size,current_time_out):

	global Bluetooth_sock_
	Bluetooth_sock_.settimeout(current_time_out)
	data_read = []
	for i in range(5):
		try:
			data_received = Bluetooth_sock_.recv(current_data_size)
			if data_received:                
				data_len = len(data_received)
				for j in range(data_len):				
					data_read.append(data_received[j])
		except bluetooth.btcommon.BluetoothError:
			pass
	return data_read		

	#socket.close()	

## Function which receive general info from the microcontroller
def Bluetooth_Get_INFO():	

	global Bluetooth_sock_
	data_to_send = [63, 13, 10]
	Bluetooth_sock_.send(bytes(data_to_send))
	Bluetooth_sock_.settimeout(5.0)
	data_read = ""
	for i in range(5):
		try:
			data_received = Bluetooth_sock_.recv(500)
			if data_received:
				#Bluetooth_sock_.flush()
				data_read = data_read + data_received.decode()
		except bluetooth.btcommon.BluetoothError:
			pass
	print()
	print(data_read)

## Function which returns the current ID of the device used by the bluetooth and acoustic modem communication
def Get_Device_ID(communication_mode):

	data_out = [42, 58, 0, 58, 13, 10]
	data_in = [48]
	current_data_size = 20
	current_time_out = 0.05
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which returns the current ID of the device used by the bluetooth acoustic modem communication
def Set_Device_ID(communication_mode, choice):

	data_out = [42, 58, 1, 58, choice, 58, 13, 10]
	data_in = [48]
	current_data_size = 20
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out)
	return data_in
