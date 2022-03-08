
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

if __name__ == "__main__":
    
	system('clear')
	print("Robotic Fish, DEMO 1 Slave Fish")
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
    current_data_size = 20
    current_time_out = 0.2
	while(open_cycle == 1):
		
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
		if (data_in[0] == 91) # Received data from acoustic modem
			if (data_in[2] == 'B')
				motor_1_dir = data_in[8]
				motor_1_speed = data_in[9]
				motor_2_dir = data_in[10]
				motor_2_speed = data_in[11]
				motor_3_dir = data_in[12]
				motor_3_speed = data_in[13]
				motor_direction = [motor_1_dir, motor_2_dir, motor_3_dir]
				motor_speed = [motor_1_speed, motor_2_speed, motor_3_speed]
				Motor.Control_All_Motors(Communication_Mode,motor_direction,motor_speed)

	print(colored("See you soon", 'blue'))
