
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

## @package BMS_Functions
#
#  This python module contains all the functions to set or get battery management system parameters or give it commands 
#  
#  The high-level microprocessor (e.g. Raspberry PI) can communicate with the 
#  the microcontroller inside the robotic fish by the serial port or throught the bluetooth.
 
## Documentation for a function.
#
#  More details.
#

import Microcontroller_Manager_Serial as Serial
import Microcontroller_Manager_Bluetooth as Bluetooth

## Function which get the current cell voltage value
def Cell_Get_Voltage_Value(communication_mode, choice):
	data_out = [51, 58, choice, 58, 13, 10]
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

## Function which get the current battery temperature value
def Battery_Get_Temperature_Value(communication_mode):
	data_out = [52, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in		

## Function which get the current of the battery
def Battery_Get_Current_Value(communication_mode):
	data_out = [53, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which get the autobalancing status of the cells
def Battery_Get_Autobalancing_Status(communication_mode):
	data_out = [54, 58, 0, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which set the autobalancing status of the cells
def Battery_Set_Autobalancing_Status(communication_mode, choice):
	data_out = [54, 58, 1, 58, choice, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which get the overvoltage and the undervoltage thresholds for the cells
def Battery_Get_Voltage_Thresholds(communication_mode):
	data_out = [55, 58, 0, 58, 13, 10]
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

## Function which set the overvoltage and the undervoltage thresholds for the cells
def Battery_Set_Voltage_Thresholds(communication_mode, over_v, over_t, under_v, under_t):
	
	aux_var_1 = np.int8((over_v>>8))
	aux_var_2 = np.int8((over_v<<8)>>8)
	aux_var_3 = np.int8((under_v>>8))
	aux_var_4 = np.int8((under_v<<8)>>8)

	data_out = [55, 58, 1, 58, aux_var_1, aux_var_2, 58, over_t, 58, aux_var_3, aux_var_4, 58, under_t, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which get the min and max temperature thresholds during the discharging and the charging of the battery
def Battery_Get_Temperature_Thresholds(communication_mode):
	data_out = [56, 58, 0, 58, 13, 10]
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

## Function which set the min and max temperature thresholds during the discharging and the charging of the battery
def Battery_Set_Temperature_Thresholds(communication_mode, min_d, max_d, min_c, max_c):
	
	aux_var_1 = np.int8(min_d + 127)
	aux_var_2 = np.int8(max_d + 127)
	aux_var_3 = np.int8(min_c + 127)
	aux_var_4 = np.int8(max_c + 127)

	data_out = [55, 58, 1, 58, aux_var_1, aux_var_2, 58, over_t, 58, aux_var_3, aux_var_4, 58, under_t, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in			

## Function which get the discharging overcurrent, the charging overcurrent and the short circuit current thresholds
def Battery_Get_Current_Thresholds(communication_mode):
	data_out = [57, 58, 0, 58, 13, 10]
	data_in = [48]
	current_data_size = 30
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which set the discharging overcurrent, the charging overcurrent and the short circuit current thresholds
def Battery_Set_Current_Thresholds(communication_mode, d_over_v, d_over_t, c_over_v, c_over_t, shortc_v, shortc_t):
	
	aux_var_1 = np.int16((d_over_v>>8))
	aux_var_2 = np.int8((d_over_v<<8)>>8)
	aux_var_3 = np.int16((c_over_v>>8))
	aux_var_4 = np.int8((c_over_v<<8)>>8)
	aux_var_5 = np.int16((shortc_v>>8))
	aux_var_6 = np.int8((shortc_v<<8)>>8)

	data_out = [57, 58, 1, 58, aux_var_1, aux_var_2, 58, d_over_t, 58, aux_var_3, aux_var_4, 58, c_over_t, 58, aux_var_5, aux_var_6, 58, shortc_t, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which get the automatic charging mode status and the current charging status of the battery
def Battery_Get_Automatic_Charging_Status(communication_mode):
	data_out = [58, 58, 0, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in

## Function which set the automatic charging mode for the battery
def Battery_Set_Automatic_Charging_Status(communication_mode, choice):
	data_out = [58, 58, 1, 58, choice, 58, 13, 10]
	data_in = [48]
	current_data_size = 10
	current_time_out = 0.5
	if (communication_mode == 2):
		Bluetooth.Bluetooth_Send_Data(bytes(data_out))
		data_in = Bluetooth.Bluetooth_Receive_Data(current_data_size,current_time_out)
	else:		
		Serial.Serial_Port_Send_Data(data_out)
		data_in = Serial.Serial_Port_Receive_Data(current_data_size,current_time_out) 
	return data_in