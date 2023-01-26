#!/usr/bin/env python
"""
#title           :multiserial_pparsing.py
#description     :Python Script to read multi-serial-port + parsing to json format
#author          :Nicholas Putra Rihandoko
#date            :2022/12/12
#version         :0.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""
import rospy
from std_msgs.msg import String
import serial
import pynmea2 #library for parsing GPS NMEA format
import time #library for time
#import datetime #library for date & time
import ekf

mode = 0
dt = 0
lat = 0
lon = 0
sats = 0
odo_VL = 0.1
odo_VR = 0.1
cond = "trouble: init. USB"

def talker():
    global mode, dt, lat, lon, sats, odo_VL, odo_VR, cond

    odom=serial.Serial("/dev/ttyACM0",9600,timeout=1)
    odom.baudrate=57600

    odom.reset_input_buffer()

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cond = "program running"
        try:
            line_odom=odom.readline().decode('utf-8').split(',')
            parsed = [x.rstrip() for x in line_odom]
            if len(parsed) > 2:
                odo_VL = float(parsed[0])
                odo_VR = float(parsed[1])
                mode = float(parsed[2])
        except:
            cond = "no odometry meas."
            print(cond)
            pass
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass