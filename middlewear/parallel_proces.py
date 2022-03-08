#!/usr/bin/env python3

from sys import ps1
import rospy
import numpy as np
from os import system
import time
import threading
import Microcontroller_Manager_Serial as Serial
import IMU_Functions as IMU
import Pressure_Functions as Pressure
import Modem_Functions as Modem
import threading 
import time

from time import sleep
from std_msgs.msg import Float32
from std_msgs.msg import String

from __future__ import print_function
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse

class RepeatedTimer(object):
  def __init__(self, interval, function, *args, **kwargs):
    self._timer = None
    self.interval = interval
    self.function = function
    self.args = args
    self.kwargs = kwargs
    self.is_running = False
    self.next_call = time.time()
    self.start()

  def _run(self):
    self.is_running = False
    self.start()
    self.function(*self.args, **self.kwargs)

  def start(self):
    if not self.is_running:
      self.next_call += self.interval
      self._timer = threading.Timer(self.next_call - time.time(), self._run)
      self._timer.start()
      self.is_running = True

  def stop(self):
    self._timer.cancel()
    self.is_running = False



mutex = threading.Lock()

Communication_Mode_ = 0


pub_pressure = rospy.Publisher('depth',Float32,queue_size=1)
pub_modem = rospy.Publisher('modem_data',Float32,queue_size=1)


P0 = 1.01325 #Default Pressure


def callback(data):
    global P0
    rospy.init_node('talker', anonymous=True)
    mutex.acquire()
    while not rospy.is_shutdown():
        
        try:
            data_received_pressure = Pressure.Pressure_Get_Final_Values(1,1)
            data_received_imu = IMU.IMU_Get_Values(1, 1)
            P1 = (np.int16((data_received_pressure[6]<<24) | (data_received_pressure[7]<<16) | (data_received_pressure[8]<<8) | (data_received_pressure[9])))/10000
            P0 = (np.int16((data_received_pressure[6]<<24) | (data_received_pressure[7]<<16) | (data_received_pressure[8]<<8) | (data_received_pressure[9])))/10000

            P = P1 - P0 # Relative Measured Pressure
            pressure = P
            pub_pressure.publish(pressure)
        except:
             print ("pressure not obtained")

    def callback_modem(data_in):

        data_in = Serial.Serial_Port_Receive_Data(20,0.2) 
        if (data_in[0] == 91) # Received data from acoustic modem
        rt = RepeatedTimer(1, data_in, "Recieved Data") # it auto-starts, no need of rt.start()
        modem_data= data_in
        pub_modem.publish(modem_data)

        try:
            sleep(0.3) # your long-running job goes here...
        finally:
            rt.stop() # better in a try/finally block to make sure the program ends!



    def talker():
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()




    def handle_add_two_ints(req):
        print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
        return AddTwoIntsResponse(req.a + req.b)

    def add_two_ints_server():
        rospy.init_node('add_two_ints_server')
        s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
        print("Ready to add two ints.")
        rospy.spin()

    if __name__ == "__main__":
        add_two_ints_server()