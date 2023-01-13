"""
#!/usr/bin/env python3
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

import serial
import pynmea2 #library for parsing GPS NMEA format
import time #library for time
#import datetime #library for date & time
import ekf
import json

time_prev = time.time()
odom=serial.Serial("/dev/ttyACM0",9600,timeout=1)
odom.baudrate=9600
gps=serial.Serial("/dev/ttyUSB0",9600,timeout=1)
gps.baudrate=9600

odom.reset_input_buffer()
gps.reset_input_buffer()

mode = None
dt = None
lat = None
lon = None
sats = None
odo_VL = None
odo_VR = None

init = True
while init:
    line_gps=gps.readline().decode('utf-8', errors='replace').strip()
    if '$GNGGA' in line_gps:
        msg = pynmea2.parse(line_gps)
        lat = msg.latitude
        lon = msg.longitude

        line_odom=odom.readline().decode('utf-8').split(',')
        parsed = [x.rstrip() for x in line_odom]
        if len(parsed) > 1:
            odo_VL = 0 #float(parsed[0])
            odo_VR = 1 #float(parsed[1])
            init = False

while True: # Run forever
    line_odom=odom.readline().decode('utf-8').split(',')
    parsed = [x.rstrip() for x in line_odom]
    if len(parsed) > 1:
        odo_VL = float(parsed[0])
        odo_VR = float(parsed[1])
        #print(parsed)
        mode = 1

    line_gps=gps.readline().decode('utf-8', errors='replace').strip()
    if '$GNGGA' in line_gps:
        msg = pynmea2.parse(line_gps)
        lat = msg.latitude
        lon = msg.longitude
        sats = msg.num_sats
        #hdop = msg.horizontal_dil
        #print(lat)
    else:
        mode = 0
    
    time_now = time.time()
    dt = time_now-time_prev
    time_prev = time_now

    # maps = [latitude, longitude, heading]
    maps = ekf.filtering(mode,dt,lat,lon,odo_VL,odo_VR)
    maps_obj = {
        "latitude" : maps[0],
        "longitude" : maps[1],
        "heading" : maps[0],
        "satelite" : sats
    }
    maps_json = json.dumps(maps_obj)
    print(maps_json)
    