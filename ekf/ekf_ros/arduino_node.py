#!/usr/bin/env python
"""
#title           :arduino_node.py
#description     :Python Script to read serial-port from Arduino + publish to ROS topics
#author          :Nicholas Putra Rihandoko
#date            :2023/01/27
#version         :1.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import serial

def odometry():
    cond = "trouble: init. USB"
    # setup serial connection
    try:
        odom=serial.Serial("/dev/ttyACM0",57600,timeout=1)
        odom.baudrate=57600
        odom.reset_input_buffer()
    except:
        print(cond)

    # Create a new topic to publish data from Arduino
    wheel_speed = rospy.Publisher('wheel_speed', Twist, queue_size=10)
    ekf_state = rospy.Publisher('ekf_state', UInt8, queue_size=10)

    # Create the message
    odom_msg = Twist()
    mode = UInt8()

    # initializing the publisher node
    rospy.init_node('arduino_node', anonymous=True)

    # set the rate at which values will be published 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cond = "reading serial"
        try:
            # read serial line and decode it into variables
            line_odom=odom.readline().decode('utf-8').split(',')
            parsed = [x.rstrip() for x in line_odom]

            # asssign the massage's value
            odom_msg.linear.x = float(parsed[0]) # left velocity
            odom_msg.angular.z = float(parsed[1]) # right velocity
            mode = float(parsed[2])
            print(cond)
        except:
            cond = "no odometry meas."
            print(cond)
            pass
        
        # Publish the message
        #rospy.loginfo(odom_msg)
        wheel_speed.publish(odom_msg)
        ekf_state.publish(mode)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        odometry()
    except rospy.ROSInterruptException:
        pass