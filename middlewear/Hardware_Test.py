
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

## Hardware Test Version: var which contains the current version of the script
Script_Version = 0.1

if __name__ == "__main__":
    
	Communication_Mode_ = 0
	open_cycle = 1
	#LED_NUMBER = 10

	system('clear')
	print("Robotic Fish, hardware test version: ", Script_Version)
	cycle = 1
	while cycle:
		print()
		print("How do you want communicate? ")
		print("1 - Serial Communication")
		print("2 - Bluetooth")
		input_var = int(input())
		if input_var == 1: # Serial
			Communication_Mode = 1
			cycle = 0
		elif input_var == 2: # Bluetooth
			Communication_Mode = 2
			cycle = 0

	if Communication_Mode == 1:
		print("Start serial port scan...")
		Serial.Serial_Port_Scan()
		Serial.Serial_Port_Open()

	elif Communication_Mode == 2:
		Bluetooth.Bluetooth_Device_Scan()
		Bluetooth.Bluetooth_Device_Open()

	while(open_cycle == 1):
		# Print the Menu
		system('clear')
		print()
		print(colored("Choice one operation:" ,'red'))
		print("1 - Exit")
		print(colored("System commands", 'green'))
		print("41 - Restart the microcontroller")
		print("42 - Device ID")
		#print("43 - Turn on/off devices")		
		print(colored("Battery commands", 'green'))
		print("51 - Get battery voltage")
		#print("52 - Get battery temperature")
		#print("53 - Get battery current")
		#print("54 - Get or Set the autobalancing mode")
		#print("55 - Get or Set the voltage threshold to detect undervoltage or overvoltage status")
		#print("56 - Get or Set temperature threshold during discharging and charging")
		#print("57 - Get or Set the thresholds to detect discharge overcurrent, charge overcurrent and short circuti current")		
		#print("58 - Get or Set the automatic charge mode and the current charging status")
		print(colored("General commands", 'green'))
		print("63 - Get INFO from the microcontroller")
		print(colored("Serial port commands", 'green'))
		print("71 - Get or set the current BAUDRATE")
		print(colored("IMU commands", 'green'))
		print("81 - Accelerometers full-scale")
		print("82 - Gyroscopes full-scale")
		print("83 - Read data from IMU")
		print("84 - Read the magnetometer sensivity adjustment parameters")
		print(colored("Acoustic Modem commands", 'green'))
		print("91 - Send data throught acoustic modem")
		print(colored("Pressure/Temperature sensor commands", 'green'))
		print("101 - Get the calibraton parameters")
		print("102 - Get the final values from the pressure/temperature sensor")
		print("103 - Get the raw values from the pressure/temperature sensor")
		print(colored("Turbidity sensor commands", 'green'))
		print("111 - Get turbidity sensor current value")
		print(colored("Motors commands", 'green'))
		print("121 - Move one motor")
		print("122 - Stop all motors")
		print(colored("Ecosounder commands", 'green'))
		print("131 - Get measurement from the Ecosounder")
		print(colored("LEDs Strip", 'green'))
		print("141 - Set the LEDs status")	
		print()

		input_var = int(input())
		if (input_var == 1):
			open_cycle = 0
		elif (input_var == 41):
			Microcontroller.Microcontroller_Reset(Communication_Mode)
			open_cycle = 0
		elif (input_var == 42):
			print("Press 0 to get the current value or 1 to set")
			input_var = int(input())
			if (input_var == 0):
				data_received = Bluetooth.Get_Device_ID(Communication_Mode)
				print("Current ID Value: ", data_received[2])
			if (input_var == 1):
				print(colored("Write the new value (between 1 and 100)", 'green'))
				input_var = int(input())
				if ((input_var > 0) and (input_var < 101)):
					data_received = Bluetooth.Set_Device_ID(Communication_Mode, input_var)
			print(colored("Raw data received: ", 'red'), data_received)
		elif (input_var == 43):
			print(colored("Choice which device do you want manage",'red'))
			print("0 - All devices")
			print("1 - Blue Light module")
			print("2 - IMU module")
			print("3 - Ecosounder module")
			print("4 - ESENSE module")
			print("5 - Acoustic modem")
			print("6 - Pressure/Temperature sensor")
			print("7 - Turbidity sensor")
			print("8 - uProcessor Board")
			print(colored("!If you are turning off this device then I can not turn on it again, but YOU NEED TO RESTART the microcontroller", 'red'))
			device = int(input())
			if ((device > -1) and (device < 9)): 
				print("Press 0 to turn off or 1 to turn on")
				choice = int(input())
				if ((choice == 0) or (choice == 1)):
					data_received = Microcontroller.Microprocessor_Port_Expanded_Switch(Communication_Mode, choice, device)
					print(colored("Raw data received: ", 'red'), data_received)
				else:
					print("Wrong choice")
			else:
				print("Wrong choice")					
		# Battery
		elif (input_var == 51):
			#print("Which cell do you want measure?(between 0 and 3, type 0 for all)")
			#input_var = int(input())
			input_var = 0
			if ((input_var > -1) and (input_var < 4)):
				data_received = BMS.Cell_Get_Voltage_Value(Communication_Mode, input_var)
				if (input_var == 0):
					#print("Cell 1: ", np.int16((data_received[2]<<8) | (data_received[3])), "mV")
					#print("Cell 2: ", np.int16((data_received[4]<<8) | (data_received[5])), "mV")
					#print("Cell 3: ", np.int16((data_received[6]<<8) | (data_received[7])), "mV")
					#print("Battery: ", np.int16((data_received[8]<<8) | (data_received[9])), "mV")
					print("Battery: ", np.int16((data_received[2]<<8) | (data_received[3])), "mV")
				else:
					print("Cell " , input_var, ": ", np.int16((data_received[2]<<8) | (data_received[3])), "mV")
				print(colored("Raw data received: ", 'red'), data_received)
			else:
				print("Wrong choice")
		elif (input_var == 52):
			data_received = BMS.Battery_Get_Temperature_Value(Communication_Mode)
			print(colored("Raw data received: ", 'red'), data_received)
			print("Temperature " , data_received[2], "°C")
		elif (input_var == 53):
			data_received = BMS.Battery_Get_Current_Value(Communication_Mode)
			print(colored("Raw data received: ", 'red'), data_received)
			print("Current: " , np.int16((data_received[2]<<8) | (data_received[3])), "mA")
		elif (input_var == 54):
			print("Type 0 to get the autobalancing status or 1 to set")
			input_var = int(input())
			if ((input_var == 0) or (input_var == 1)):
				if (input_var_ == 1):
					print("Type 0 to disable the autobalancing or 1 to enable")
					choice = int(input())
					if ((choice == 0) or (choice == 1)):
						data_received = BMS.Battery_Set_Autobalancing_Status(Communication_Mode, choice)
						print(colored("Raw data received: ", 'red'), data_received)
					else:
						print("Wrong choice")		
				else:
					data_received = BMS.Battery_Get_Autobalancing_Status(Communication_Mode)
					print(colored("Raw data received: ", 'red'), data_received)	
					if (data_received[2]) == 0:
						print("Autobalancing status: DISABLED")
					else:
						print("Autobalancing status: ENABLED") 	
			else:
				print("Wrong choice")
		elif (input_var == 55):
			print("Type 0 to get the overvoltage and the undervoltage thresholds or 1 to set them")
			input_var = int(input())
			if ((input_var == 0) or (input_var == 1)):
				if (input_var_ == 1):
					print("Desired overvoltage value for each cell? (in mV)")
					over_v = int(input())
					print("Desired overvoltage delay for each cell (in s) ")
					over_t = int(input())
					print("Desired undervoltage value for each cell? (in mV)")
					under_v = int(input())
					print("Desired undervoltage delay for each cell (in s) ")
					under_t = int(input())
					data_received = BMS.Battery_Set_Voltage_Thresholds(Communication_Mode, over_v, over_t, under_v, under_t)
					print(colored("Raw data received: ", 'red'), data_received)
				else:
					data_received = BMS.Battery_Get_Voltage_Thresholds(Communication_Mode)
					print("Overvoltage threshold: ", np.int16((data_received[2]<<8) | (data_received[3])), "mV; ", data_received[5], "s")
					print("Undervoltage threshold: ", np.int16((data_received[7]<<8) | (data_received[8])), "mV; ", data_received[10], "s")
					print(colored("Raw data received: ", 'red'), data_received)	
			else:
				print("Wrong choice")
		elif (input_var == 56):
			print("Type 0 to get the discharging and the charging temperature thresholds or 1 to set them")
			input_var = int(input())
			if ((input_var == 0) or (input_var == 1)):
				if (input_var_ == 1):
					print("Desired min temperature value during the discharging? (in °C)")
					min_d = int(input())
					print("Desired min temperature value during the discharging? (in °C)")
					max_d = int(input())
					print("Desired min temperature value during the discharging? (in °C)")
					min_c = int(input())
					print("Desired min temperature value during the discharging? (in °C)")
					max_c = int(input())
					data_received = BMS.Battery_Set_Temperature_Thresholds(Communication_Mode, min_d, max_d, min_c, max_c)
					print(colored("Raw data received: ", 'red'), data_received)
				else:
					data_received = BMS.Battery_Get_Temperature_Thresholds(Communication_Mode)
					print("Min discharging temperature threshold: ", data_received[2]- 127)
					print("Max discharging temperature threshold: ", data_received[4]- 127)
					print("Min charging temperature threshold: ", data_received[6]- 127)
					print("Max charging temperature threshold: ", data_received[8]- 127)					
					print(colored("Raw data received: ", 'red'), data_received)	
			else:
				print("Wrong choice")
		elif (input_var == 57):
			print("Type 0 to get the discharging overcurrent, the charging overcurrent and the short circuit current thresholds or 1 to set them")
			input_var = int(input())
			if ((input_var == 0) or (input_var == 1)):
				if (input_var_ == 1):
					print("Desired discharging overcurrent value? (in mA)")
					d_over_v = int(input())
					print("Desired discharging overcurrent delay? (in ms)")
					d_over_t = int(input())
					print("Desired charging overcurrent value? (in mA)")
					c_over_v = int(input())
					print("Desired charging overcurrent delay? (in ms)")
					c_over_t = int(input())
					print("Desired short circuit current value? (in mA)")
					shortc_v = int(input())
					print("Desired short circuit delay? (in us)")
					shortc_t = int(input())
					data_received = BMS.Battery_Set_Current_Thresholds(Communication_Mode, d_over_v, d_over_t, c_over_v, c_over_t, shortc_v, shortc_t)
					print(colored("Raw data received: ", 'red'), data_received)
				else:
					data_received = BMS.Battery_Get_Current_Thresholds(Communication_Mode)
					print("Discharging overcurrent thresholds: ", np.int16((data_received[2]<<8) | (data_received[3])), "mA; ", data_received[5], "ms")
					print("Charging overcurrent thresholds: ", np.int16((data_received[7]<<8) | (data_received[8])), "mA; ", data_received[10], "ms")
					print("Short circuit current thresholds: ", np.int16((data_received[12]<<8) | (data_received[13])), "mA; ", data_received[15], "us")			    	
					print(colored("Raw data received: ", 'red'), data_received)
			else:
				print("Wrong choice")		
		elif (input_var == 58):
			print("Type 0 to get the status of automatic charge mode or 1 to set")
			input_var = int(input())
			if ((input_var == 0) or (input_var == 1)):
				if (input_var_ == 1):
					print("Type 0 to disable the automatic charge mode or 1 to enable")
					choice = int(input())
					if ((choice == 0) or (choice == 1)):
						data_received = BMS.Battery_Set_Automatic_Charging_Status(Communication_Mode, choice)
						print(colored("Raw data received: ", 'red'), data_received)
					else:
						print("Wrong choice")		
				else:
					data_received = BMS.Battery_Get_Automatic_Charging_Status(Communication_Mode)
					print(colored("Raw data received: ", 'red'), data_received)	
					if (data_received[2]) == 0:
						print("Automatic charging mode: DISABLED")
					else:
						print("Automatic charging mode: ENABLED")
					if (data_received[4]) == 0:
						print("Current battery status: DISCHARGING")
					else:
						print("Current battery status: CHARGING")			
			else:
				print("Wrong choice")			
		
		# System
		elif (input_var == 63):
			if (Communication_Mode == 2):
				Bluetooth.Bluetooth_Get_INFO()
			else:	
				Serial.Serial_Get_INFO()	
		elif (input_var == 71):
			print("Press 0 to get the current value or 1 to set")
			input_var = int(input())
			if (input_var == 0):
				data_received = Serial.Get_BAUDRATE(Communication_Mode)
				current_baudrate = (data_received[2] << 24) | (data_received[3] << 16) | (data_received[4] << 8) | (data_received[5]) 
				print("Current Baudrate: ", current_baudrate)
			if (input_var == 1):
				print(colored("Choose the new value", 'green'))
				print("1 - 9600 bits/s")
				print("2 - 38400 bits/s")
				print("3 - 115200 bits/s")
				print("4 - 230400 bits/s")
				print("5 - 460800 bits/s")
				print("6 - 921600 bits/s")
				input_var = int(input())
				if ((input_var > 0) and (input_var < 7)):
					data_received = Serial.Set_BAUDRATE(Communication_Mode, input_var)
			print(colored("Raw data received: ", 'red'), data_received)
		elif (input_var == 81):
			print("Press 0 to get the current value or 1 to set")
			input_var = int(input())
			if (input_var == 0):
				data_received = IMU.IMU_Get_Accelerometer_Full_Scale(Communication_Mode)
				if data_received[2] == 0:
					print("Accelerometers full-scale range : +-2g")
				elif data_received[2] == 8:
					print("Accelerometers full-scale range : +-4g")				
				elif data_received[2] == 16:	
					print("Accelerometers full-scale range : +-8g")
				elif data_received[2] == 24:	
					print("Accelerometers full-scale range : +-16g")
				else:
					print("Accelerometers full-scale range : unknown")
			if (input_var == 1):
				print(colored("Choose the new value", 'green'))
				print("1 - +-2g")
				print("2 - +-4g")
				print("3 - +-8g")
				print("4 - +-16g")
				input_var = int(input())
				if ((input_var > 0) and (input_var < 5)):
					data_received = IMU.IMU_Set_Accelerometer_Full_Scale(Communication_Mode, input_var)
			print(colored("Raw data received: ", 'red'), data_received)	
		elif (input_var == 82):
			print("Press 0 to get the current value or 1 to set")
			input_var = int(input())
			if (input_var == 0):
				data_received = IMU.IMU_Get_Gyroscope_Full_Scale(Communication_Mode)
				if data_received[2] == 0:
					print("Gyroscopes full-scale range : +-250°/s")
				elif data_received[2] == 8:
					print("Gyroscopes full-scale range : +-500°/s")				
				elif data_received[2] == 16:	
					print("Gyroscopes full-scale range : +-1000°/s")
				elif data_received[2] == 24:	
					print("Gyroscopes full-scale range : +-2000°/s")
				else:
					print("Gyroscopes full-scale range : unknown")
			if (input_var == 1):
				print(colored("Choose the new value", 'green'))
				print("1 - +-250°/s")
				print("2 - +-500°/s")
				print("3 - +-1000°/s")
				print("4 - +-2000°/s")
				input_var = int(input())
				if ((input_var > 0) and (input_var < 5)):
					data_received = IMU.IMU_Set_Gyroscope_Full_Scale(Communication_Mode, input_var)
			print(colored("Raw data received: ", 'red'), data_received)
		elif (input_var == 83):
			print("Which value do you want read?")
			print("1 - Complete Read")
			print("2 - Accelerometers")
			print("3 - IMU Temperature")
			print("4 - Gyroscopes")
			print("5 - Magnetometer")
			input_var = int(input())
			if ((input_var > 0) and (input_var < 6)):
				data_received = IMU.IMU_Get_Values(Communication_Mode, input_var)
			print(colored("Raw data received: ", 'red'), data_received)
			if (input_var == 1):
				print("Complete Read:")
				print("Acc_x: ", np.int16((data_received[2]<<8) | (data_received[3])))
				print("Acc_y: ", np.int16((data_received[4]<<8) | (data_received[5])))
				print("Acc_z: ", np.int16((data_received[6]<<8) | (data_received[7])))
				print("Temperature: ", np.int16((data_received[8]<<8) | (data_received[9])))
				print("Gyro_x: ",    np.int16((data_received[10]<<8) | (data_received[11])))
				print("Gyro_y: ",    np.int16((data_received[12]<<8) | (data_received[13])))
				print("Gyro_z: ",    np.int16((data_received[14]<<8) | (data_received[15])))
				print("Mag_x: ",     np.int16((data_received[16]<<8) | (data_received[17])))
				print("Mag_y: ",     np.int16((data_received[18]<<8) | (data_received[19])))
				print("Mag_z: ",     np.int16((data_received[20]<<8) | (data_received[21])))
			if (input_var == 2):
				print("Accelerometers Read:")
				print("Acc_x: ",       np.int16((data_received[2]<<8) | (data_received[3])))
				print("Acc_y: ",       np.int16((data_received[4]<<8) | (data_received[5])))
				print("Acc_z: ",       np.int16((data_received[6]<<8) | (data_received[7])))
			if (input_var == 3):
				print("Temperature Read:")
				print("Temperature: ", np.int16((data_received[2]<<8) | (data_received[3])))
			if (input_var == 4):
				print("Gyroscopes Read:")
				print("Gyro_x: ",      np.int16((data_received[2]<<8)  | (data_received[3])))
				print("Gyro_y: ",      np.int16((data_received[4]<<8) | (data_received[5])))
				print("Gyro_z: ",      np.int16((data_received[6]<<8) | (data_received[7])))
			if (input_var == 5):
				print("Magnetometer Read:")
				print("Mag_x: ",      np.int16((data_received[2]<<8)  | (data_received[3])))
				print("Mag_y: ",      np.int16((data_received[4]<<8)  | (data_received[5])))
				print("Mag_z: ",      np.int16((data_received[6]<<8)  | (data_received[7])))	
		elif (input_var == 84):
			data_received = IMU.IMU_Get_Magnetometer_Parameters(Communication_Mode)
			print("ASA_x: ", data_received[2])
			print("ASA_y: ", data_received[3])
			print("ASA_z: ", data_received[4])
			print(colored("Raw data received: ", 'red'), data_received)	
		elif (input_var == 91): # Send data throught acoustic modem
			print("Insert the receiver ID (between 0 and 100), type 0 for a broadcast data message")
			input_var = int(input())
			if ((input_var > -1) and (input_var < 101 )): 
				receiver_ID = input_var
				print("How much byte to you want to send? (up to 7)")
				input_var = int(input())
				if ((input_var > 0) and (input_var < 8)):
					data_to_send = []
					data_size = input_var
					print("Type the byte to send in ASCII code (e.g. for 'A' type 65)")
					for i in range (data_size):
						print("Type the" , i, "byte")
						input_var = int(input())
						data_to_send.append(input_var)
					Modem.Send_Data(Communication_Mode, receiver_ID, data_size, data_to_send)
				else:
					print("Wrong number of bytes to send")	
			else:
				print("Wrong Device ID")				
		elif (input_var == 101):
			data_received = Pressure.Pressure_Get_Calibration_Values(Communication_Mode)
			print("Pressure sensivity (c1): ", np.int16((data_received[2]<<8)  | (data_received[3])))
			print("Pressure offset (c2): ", np.int16((data_received[4]<<8)  | (data_received[5])))
			print("Temperature coefficient of pressure sensivity (c3): ", np.int16((data_received[6]<<8)  | (data_received[7])))
			print("Temperature coefficient of pressure offset (c4): ", np.int16((data_received[8]<<8)  | (data_received[9])))
			print("Reference temperature (c5): ", np.int16((data_received[10]<<8)  | (data_received[11])))
			print("Temperature coefficient of temperature (c6): ", np.int16((data_received[12]<<8)  | (data_received[13])))		
			print(colored("Raw data received: ", 'red'), data_received)
		elif (input_var == 102):
			print("Which value do you want read?")
			print("1 - Complete Read")
			print("2 - Temperature")
			print("3 - Pressure")
			input_var = int(input())
			if ((input_var > 0) and (input_var < 4)):
				data_received = Pressure.Pressure_Get_Final_Values(Communication_Mode, input_var)
			if (input_var == 1):
				print("Complete Read:")
				print("Temperature: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5])))/100, "°C")
				print("Pressure: ",    (np.int16((data_received[6]<<24) | (data_received[7]<<16) | (data_received[8]<<8) | (data_received[9])))/10000, "Bar") 
			elif (input_var == 2):
				print("Temperature: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5])))/100, "°C")
			elif (input_var == 3):
				print("Pressure: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5])))/10000, "Bar")
			print(colored("Raw data received: ", 'red'), data_received)
		elif (input_var == 103):
			print("Which value do you want read?")
			print("1 - Complete Read")
			print("2 - Temperature")
			print("3 - Pressure")
			input_var = int(input())
			if ((input_var > 0) and (input_var < 4)):
				data_received = Pressure.Pressure_Get_Raw_Values(Communication_Mode, input_var)
			if (input_var == 1):
				print("Complete Read:")
				print("Temperature: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5]))))
				print("Pressure: ",    (np.int16((data_received[6]<<24) | (data_received[7]<<16) | (data_received[8]<<8) | (data_received[9])))) 
			elif (input_var == 2):
				print("Temperature: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5]))))
			elif (input_var == 3):
				print("Pressure: ", (np.int16((data_received[2]<<24) | (data_received[3]<<16) | (data_received[4]<<8) | (data_received[5]))))
			print(colored("Raw data received: ", 'red'), data_received)	
		elif (input_var == 111):
			data_received = Turbidity.Get_Value(Communication_Mode)
			print("Current Value (mV): ", (np.int16((data_received[2]<<8) | (data_received[3]))))
			print(colored("Raw data received: ", 'red'), data_received)	
		elif (input_var == 121):
			print("Which motor do you want control (1-3)?")
			input_var = int(input())
			if ((input_var > 0) and (input_var < 4)):
				motor_number = input_var
				print("Which direction? (0 = Clockwise, 1 = Counterclockwise)")
				input_var = int(input())
				if ((input_var == 0) or (input_var == 1)):
					motor_direction = input_var
					print("Which speed? (0-100)")
					input_var = int(input())
					if ((input_var > 0) and (input_var < 101)):
						motor_speed = input_var
						Motor.Control_One_Motor(Communication_Mode,motor_number,motor_direction,motor_speed)
					else:
						print("Wrong choice! It must be between 0 and 100") 
				else:
					print("Wrong choice! It must be 0 or 1") 
			else:
				print("Wrong choice! It must be 1,2 or 3")
		elif (input_var == 122):
			motor_direction = [0, 0, 0]
			motor_speed = [0, 0, 0]
			Motor.Control_All_Motors(Communication_Mode,motor_direction,motor_speed)
		elif (input_var == 131):
			data_received = Bathimetry.Get_Value(Communication_Mode)
			print("Current Value: ", (np.uint16((data_received[2]<<8) | (data_received[3]))), "cm")
			print(colored("Raw data received: ", 'red'), data_received)		
		elif (input_var == 141):
			print("How much LEDs do you want manage? (Type 0 for all)")
			led_number = int(input())
			if ((led_number > -1) and (led_number < LED.LED_NUMBER)):
				if (led_number == 0):
					print("Choose a color:")
					print("0 - Black (turn off)")
					print("1 - White")
					print("2 - Purple")
					print("3 - Blue")
					print("4 - Light Blue")
					print("5 - Green")
					print("6 - Yellow")
					print("7 - Orange")					
					print("8 - Red")
					color = int(input())
					if ((color > -1) and (color < 9)):
						print("Choose the brightness (between 0 and 7)")
						brightness = int(input())
						if ((brightness > -1) and (brightness < 8)):
							data_to_send = [141, 58, 0, color, brightness, 58, 13, 10]
							LED.LEDs_Set_Status(Communication_Mode, data_to_send)
						else:
							print("Wrong choice!")	
					else:
						print("Wrong choice!")
				else:
					data_to_send = [141, 58, led_number]
					for i in range(led_number):
						print("Choose the LED")
						led = int(input())
						if ((led > 0) and (led < LED.LED_NUMBER)):
							data_to_send.append(led)
							print("Choose a color:")
							print("0 - Black (turn off)")
							print("1 - White")
							print("2 - Purple")
							print("3 - Blue")
							print("4 - Light Blue")
							print("5 - Green")
							print("6 - Yellow")
							print("7 - Orange")					
							print("8 - Red")
							color = int(input())
							if ((color > -1) and (color < 9)):
								data_to_send.append(color)
								print("Choose the brightness (between 0 and 7)")
								brightness = int(input())
								if ((brightness > -1) and (brightness < 8)):
									data_to_send.append(brightness)
								else:
									print("Wrong brightness choice!")
									break	
							else:
								print("Wrong color choice!")
								break
						else:
							print("Wrong led choice!")
							break			
					data_to_send.append(58)
					data_to_send.append(13)
					data_to_send.append(10)
					#print(data_to_send)
					LED.LEDs_Set_Status(Communication_Mode, data_to_send)		
			else:
				print("Wrong choice!")
		else:
			print("not valid choice")
		print("press RETURN to restart")
		input()

	print(colored("See you soon", 'blue'))


	
## Documentation for a function.
#
#  More details.
#