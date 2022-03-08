
# ----------------------------------------------------------------------------
# BSD 3-Clause License

# Copyright (c) 2022, Gaspare Santaera, Istituto di Biorobotica
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

## @package Hardware_Test
#
#  This is a test script. 
# 
#  This script allow the user to test and understand how to communicate with the robotic fish.
#  It is possible to access and test all the funcionalities that were implemented inside 
#  the firmware of the robotic fish.

from os import system
import Microcontroller_Manager_Serial as Serial
import Microcontroller_Manager_Bluetooth as Bluetooth
import Microcontroller_Functions as Microcontroller 
import IMU_Functions as IMU
import Turbidity_Functions as Turbidity
import Pressure_Functions as Pressure
import Motors_Functions as Motor
import Bathimetry_Functions as Bathimetry
import Modem_Functions as Modem
import BMS_Functions as BMS
import LEDs_Functions as LED
from termcolor import colored
import bluetooth
import numpy as np


## Communication_Mode: var which allow to choice how to communicate with fish, 
#  through serial port or bluetooth, default value is 1 = Serial Communication 
global Communication_Mode_

## Hardware Test Version: var which contains the current version of the script
Script_Version = 0.1

if __name__ == "__main__":
    
	Communication_Mode_ = 1
	open_cycle = 1
	go_forward_time = 4
	turn_right_time = 4
	go_up_time = 4
	go_down_time = 4
	turn_left_time = 4
	go_back_time = 4
	rest_time = 4
	forward_speed = 20
	reverse_speed = 20
	up_speed = 20
	down_speed = 20
	turn_left_speed = 20
	turn_right_speed = 20
	receiver_ID = 0
	data_size = 6

	system('clear')
	print("Robotic Fish, DEMO 1 Master Fish")
	serial_micro_ = serial.Serial()
	serial_micro_.port = 'None'
	serial_micro_.bytesize = 8
	serial_micro_.parity = 'N'
	serial_micro_.timeout = 5
	serial_micro_.timeout = 0.01 # 50 ms
	serial_micro_.baudrate = serial_deafult_value
	serial_micro_.port = "/dev/serial0"
    serial_micro_.open()
    data_received = str(serial_micro_.read(3000))
    data_received_len = len(data_received)
    time.sleep(1)
	data_to_send = [forward_speed, 0, forward_speed]
	Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)

	while(open_cycle == 1):
		
		print()
		print("Go forward for ")
		print(colored(go_forward_time, 'green'))
		print("s, at ")
		print(colored(forward_speed, 'green'))
		print("%")
		data_to_send = [1, forward_speed, 0, 0, 1, forward_speed]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(go_forward_time)

		print()
		print("Turn right for ")
		print(colored(turn_right_time, 'green'))
		print("s, at ")
		print(colored(turn_right_speed, 'green'))
		print("%")
		data_to_send = [0, turn_right_speed, 0, 0, 1, turn_right_speed]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(turn_right_time)

		print()
		print("Go up for ")
		print(colored(go_up_time, 'green'))
		print("s, at ")
		print(colored(go_up_speed, 'green'))
		print("%")
		data_to_send = [0, 0, 1, go_up_speed, 0, 0]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(go_up_time)

		print()
		print("Go down for ")
		print(colored(go_down_time, 'green'))
		print("s, at ")
		print(colored(go_down_speed, 'green'))
		print("%")
		data_to_send = [0, 0, 0, go_down_speed, 0, 0]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(go_down_time)						

		print()
		print("Turn left for ")
		print(colored(turn_left_time, 'green'))
		print("s, at ")
		print(colored(turn_left_speed, 'green'))
		print("%")
		data_to_send = [1, turn_left_speed, 0, 0, 0, turn_left_speed]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(turn_left_time)

		print()
		print("Go back for ")
		print(colored(go_back_time, 'green'))
		print("s, at ")
		print(colored(back_speed, 'green'))
		print("%")
		data_to_send = [0, back_speed, 0, 0, 0, back_speed]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(go_back_time)

		print()
		print("Rest for ")
		print(colored(rest_time, 'green'))
		print("s")
		data_to_send = [0, 0, 0, 0, 0, 0]
		Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
		time.sleep(rest_time)

	print(colored("See you soon", 'blue'))