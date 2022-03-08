#!/usr/bin/env python3

# Copyright (c) TODO all Licence


# This is a ROS Node 
#
# Description: This ros node converts an mixed input control into motor command to motors 0,1,2
# The inputs are desired depth, desired angular velocity, desired foward/backward Force
# The node requires a feedback from the pressure sensor and IMU to close the loop
# The node refers to the TAM matirx saved under /config to solve the allocation of thrust. 
#
# Publisher 
#	*Topic:  thrusters/0/input	FloatStamped
#	*Topic:  thrusters/0/input	FloatStamped
#	*Topic:  thrusters/1/input	FloatStamped
#
# Subscriber
#	*Topic:  feedback	Vector3
#		feedback.x = Angular velocity component z_body
#		feedback.y = Euler Angle pitch
#		feedback.z = Depth from pressure sensor
#
#	*Topic:  cmd	Vector3
#		cmd.x = Fx Foward force desired
#		cmd.y = Angular velocity desired along the z axis
#		cmd.z = Depth desired
#
# Parameteres required
#	tam	6x3 list
#	
# Note: The PID is using Real Time. A better approach would be using ROS_time
# Note: All topics addresses are relative

import rospy
import numpy as np
from simple_pid import PID
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

# Setting PIDs parameters
pid_z = PID(600/45, 45/45, 200/45, setpoint=0)
pid_z.output_limits = (-2.2, 1.6)


# Initialize Publishers
pub_motor = rospy.Publisher ('motors', Vector3, queue_size=1)

# initialize Global Variables
Fx = 0
yaw_dot = 0
zd = 0.5
limits = []
IRTAM = []
m0_old = 0
m1_old = 0

def horizontal(Fx,R,Fz):
    if Fx !=0:
        base = (Fx + np.sign(Fx))*5
        delta = R*3
        T = [base+delta,base-delta,Fz]
        
        
    
def verticalPID (z,zp):
    # Apply the vertical PID

    e = (zp + z) 
    return pid_z(e)


def update_cmd(cmd):
    # Callback to a new command - Updates the desired Fx, angular velocity and depth

    global Fx 
    global yaw_dot 
    global zd 

    Fx = cmd.x 
    yaw_dot = cmd.y 
    zd = cmd.z

def control(fb):
    # Callback to new sensor imput, contains main loop instructions.

    # Unpack feedback 
    r = fb.x            # Angular velocity Body_Frame
    pitch = fb.y        # Pitch Angle
    zp = fb.z           # Depth

    # Apply the vertical PID. Obtain Fz (stabilized_Frame)
    Fz = verticalPID (zd,zp)

    # Apply Reduced Inverted Thrust Allocation Matrix
    Thrust = horizontal(Fx,yaw_dot,Fz)
 
    # For each motor obtain 
    m0 = motor_signal(Thrust[0])
    m1 = motor_signal(Thrust[1])
    m2 = motor_signal(Thrust[2])

    # Publish all motors Output
    motor_vector = Vector3()
    
    motor_vector.x=m0
    motor_vector.y=m1
    motor_vector.z=m2
    
    pub_motor.publish(motor_vector)

def read_subscribers():
    # Subscriber
    # Initialize the node
    # subscribe to 2 topic, callback one of the other
    # Main loop on feedback
    # cmd updates the desired status and returns

    rospy.init_node('demo_control_node')
    rospy.Subscriber("feedback", Vector3, control)
    rospy.Subscriber("cmd", Vector3, update_cmd)
    rospy.spin()


if __name__ == '__main__':

    # Run subscriber, stating loop
    read_subscribers()
