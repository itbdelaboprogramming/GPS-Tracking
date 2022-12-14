"""
#!/usr/bin/env python3
#title           :odometry_serial.py
#description     :Python Script Communication between Arduino Mega and Raspberry Pi
#author          :Nicholas Putra Rihandoko
#date            :2022/12/09
#version         :0.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""

# Library
import serial #library for serial communication
import time #library for time
#import datetime #library for date & time

#initialize variable
global forward
global turning

# Serial communication
try:
    ser = serial.Serial('COM3', baudrate=9600, timeout=1)
    #print("Connected to Arduino Mega 2560")
except:
    print("Disconnected to Arduino Mega 2560")

#ser.reset_input_buffer()

# Algorithm to Arduino Mega
while True:
    try:
        #if ser.in_waiting > 0:
            # readline() will read everything until the new line character
            # decode('utf-8') will convert the data form a byte into a string of type 'utf-8'
            line = ser.readline().decode('utf-8')
        
            # take out the spaces, parse string into a list
            parsed = line.split(',')

            # rstrip() will remove trailing characters and white spaces
            parsed = [x.rstrip() for x in parsed]

            # assign parsed data to variable if the amount of data (2) matched the expected value
            if (len(parsed) > 1):

                # Timer
                #timer = datetime.datetime.now()

                # Variable
                forward = float(parsed[0])
                turning = float(parsed[1])
                
                # print the data
                #print("Time          :", timer.strftime("%Y-%m-%d %H:%M:%S"))
                #print("================================")
                print("forward [m/s] : ", str(forward))
                print("turn [rad/s] : ", str(turning))
                #print("================================")
                #print("odometry")
                time.sleep(0.5)

    except:
        #Disconnected
        #print("Arduino Mega 2560 is not sent")
        #print("================================")
        #print("")
        #time.sleep(5)
        pass