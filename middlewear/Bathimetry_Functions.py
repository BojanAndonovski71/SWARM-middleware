
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

## @package Bathimetry_Functions
#
#  This python module contains all the functions to manage the ecosounder used for the bathimetry 
#  
#  The high-level microprocessor (e.g. Raspberry PI) can communicate with the 
#  the microcontroller inside the robotic fish by the serial port or throught the bluetooth.
 
## Documentation for a function.
#
#  More details.
#

import Microcontroller_Manager_Serial as Serial
import Microcontroller_Manager_Bluetooth as Bluetooth

## Function which get the current measurements from the ecosounder sensor
def Get_Value(communication_mode):
	data_out = [131, 13, 10]
	data_in = [48]
	current_data_size = 20
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in
