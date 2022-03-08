#!/usr/bin/env python

# Copyright (c) TODO all Licence


# This is a ROS Node 
#
# Description: This ros node recive informations from the visual servoing
# The information are converted into 2 input commands Fx and turning velocity
# Depth is set 
#  
#
# Publisher 
#	*Topic:  cmd	Vector3
#		cmd.x = Fx Foward force desired
#		cmd.y = Angular velocity desired along the z axis
#		cmd.z = Depth desired
#
# Subscriber
#	*Topic:  get_visual_servo	Vector3
#		get_visual_servo.x = Horizontal average position (normalized [-1,1] Left-Right )
#		get_visual_servo.y = Vertical average position (normalized [-1,1] Top-Bottom <-- CHECK  )
#		get_visual_servo.z = Area / Size of target : Total number of pixels
#

# Parameteres Set
#	desired_depth	= 0.5 
#	
# Note: Topics addresses are relative

import numpy as np
import rospy
from geometry_msgs.msg import Vector3

# Initialize Publisher
pub = rospy.Publisher('cmd', Vector3, queue_size=1)

# initialize global variable
last_sent_ctrl = Vector3(0,0,0.5)
timer = 0.0

def send_ctrl(ctrl_input):

    global last_sent_ctrl
    global timer

    # Unpack recived topic
    x_target = ctrl_input.x
    Area = ctrl_input.z        

    # Define Command output vector    
    cmd_output = Vector3()
    
    # Set Depth
    # For now we are not using the image info: ctrl_input.y 
    z = rospy.get_param("desired_depth")
    cmd_output.z = z


    if Area <= 5: # No target detected
        if rospy.get_time() - timer > 3:
            #search strategy
            cmd_output.x = 0.025
            cmd_output.y = 0.2
        else:
            # Maintain previous target
            cmd_output = last_sent_ctrl
    else:

        # Set turn Velocity
        cmd_output.y = - x_target * 1.2

       # Calculates distance and related foward force (We set 80 sqarepixels corresponding to 0.6 meters)
        meters = np.sqrt(160.0/Area)*0.6  # indicative distance
        Fx = (meters-0.6)*1.2            # Apply a foward force above 0.6 meters, backard force under 0.6 meters distance
        if Fx >1.3:
            Fx = 1.3
        cmd_output.x = Fx
        timer  = rospy.get_time()

    # Save last control
    last_sent_ctrl = cmd_output

    # Publish output
    pub.publish (cmd_output)


def read_servo_image():
    # Initialize node, Subscriber and Spin
    rospy.init_node('get_visual_servo')
    rospy.Subscriber("servo_img", Vector3, send_ctrl)
    rospy.spin()


if __name__ == '__main__':

    # Set depth and start loop
    rospy.set_param('desired_depth', 0.5 )   
    read_servo_image()
