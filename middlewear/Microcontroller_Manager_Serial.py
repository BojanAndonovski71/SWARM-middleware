
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

## @package Microcontroller_Manager_Serial
#
#  This python module contains all the functions to communicate with the robotic fish throught serial
#  
#  The high-level microprocessor (e.g. Raspberry PI) can communicate with the 
#  the microcontroller inside the robotic fish by the serial port or throught the bluetooth.
 
## Documentation for a function.
#
#  More details.
#

import serial
import serial.tools.list_ports
import time
import Microcontroller_Manager_Bluetooth as Bluetooth

## Function which scans and returns all the serial port connected
def Serial_Port_Scan():

	serial_deafult_value = 115200
	global serial_micro_
	serial_micro_ = serial.Serial()
	serial_micro_.port = 'None'
	serial_micro_.bytesize = 8
	serial_micro_.parity = 'N'
	serial_micro_.timeout = 2
	serial_micro_.timeout = 0.01 # 50 ms
	print("Which is the BAUDRATE?, The default value is", serial_deafult_value)
	print("1 - 9600 bit/s")
	print("2 - 38400 bit/s")
	print("3 - 115200 bit/s")
	print("4 - 230400 bit/s")
	print("5 - 460800 bit/s")
	print("6 - 921600 bit/s")
	input_var = int(input())
	if input_var == 1:
		serial_micro_.baudrate = 9600
	elif input_var == 2:		
		serial_micro_.baudrate = 38400
	elif input_var == 3:		
		serial_micro_.baudrate = 115200
	elif input_var == 4:		
		serial_micro_.baudrate = 230400
	elif input_var == 5:		
		serial_micro_.baudrate = 460800
	elif input_var == 6:		
		serial_micro_.baudrate = 921600
	else:
		print("Choice not valid, default value was chosen")
		serial_micro_.baudrate = serial_deafult_value
	print("Choice a port: ")
	print("For the raspberry the common choice is serial0")
	print("1 - ttyUSB0")
	print("2 - ttyUSB1")
	print("3 - serial0")
	print("4 - serial1")	
	connected_ports_ = list(serial.tools.list_ports.comports())
	number_of_ports_ = len(connected_ports_)
	if (number_of_ports_ > 0):	
		for Ports_N in range(number_of_ports_):
			current_port = str(connected_ports_[Ports_N][1])
			start_char = current_port.find('(') + 1 
			stop_char=current_port.find(')')    
			current_serial_port = current_port[start_char:stop_char]
			print(Ports_N+5, " ", current_serial_port)
	input_var = int(input())
	if input_var == 1:
		serial_micro_.port = "/dev/ttyUSB0"
	elif input_var == 2:
		serial_micro_.port = "/dev/ttyUSB1"
	elif input_var == 3:
		serial_micro_.port = "/dev/serial0"
	elif input_var == 4:
		serial_micro_.port = "/dev/serial1"		
	else:
		input_var = input_var - 5
		current_port = str(connected_ports_[input_var][0])
		serial_micro_.port = current_port
	print("Port chosen: ", serial_micro_.port)

## Function which opens the serial port following the chosen settings
def Serial_Port_Open():

    serial_micro_.open()
    data_received = str(serial_micro_.read(3000))
    data_received_len = len(data_received)

    time.sleep(1)

    if (serial_micro_.is_open):
        serial_micro_.timeout = 5 # For info command is better to use a long timeout
        print("Request...")
        data_out = [63, 13, 10]
        serial_micro_.write(data_out)
        data_received_raw = serial_micro_.read(3000)
        data_received_len = len(data_received_raw)
        data_received = str(data_received_raw.decode())
        print()
        print(data_received)
        serial_micro_.timeout = 0.05

## Function which sends a selected package to the microcontroller throuth the serial port
def Serial_Port_Send_Data(data_to_send):

	serial_micro_.timeout = 0.2
	serial_micro_.write(data_to_send)
	#data_received = serial_micro_.read(1000)
	#print(str(data_received.decode()))

## Function which receives a package from the microcontroller throuth the serial port
def Serial_Port_Receive_Data(current_data_size,current_time_out):

	serial_micro_.timeout = current_time_out
	data_received_raw = serial_micro_.read(current_data_size)
    #data_received = data_received_raw.decode()
	data_received_len = len(data_received_raw)
	serial_micro_.timeout = 0.01
	return data_received_raw
	#return data_received_raw, data_received_len

## Function which receive general info from the microcontroller
def Serial_Get_INFO():

    serial_micro_.timeout = 5 # For info command is better to use a long timeout
    #print("Request...")
    data_out = [63, 13, 10]
    serial_micro_.write(data_out)
    data_received_raw = serial_micro_.read(3000)
    data_received_len = len(data_received_raw)
    data_received = str(data_received_raw.decode())
    print()
    print(data_received)
    serial_micro_.timeout = 0.05

## Function which returns the current value of the BAUDRATE for the serial communication
def Get_BAUDRATE(communication_mode):

	current_data_size = 10
	current_time_out = 0.1 # For info command is better to use a long timeout
	data_out = [71, 58, 0, 58, 13, 10]
	data_in = [48]
	if (communication_mode == 2):
		current_data_size = 10
		current_time_out = 0.5
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial_Port_Send_Data(data_out)
		data_in = Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which sets the current value of the BAUDRATE for the serial communication
def Set_BAUDRATE(communication_mode, choice):

	data_out = [71, 58, 1, 58, choice, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial_Port_Send_Data(data_out)
		data_in = Serial_Port_Receive_Data(current_data_size,current_time_out) 
	#print(str(data_in))
	return data_in



