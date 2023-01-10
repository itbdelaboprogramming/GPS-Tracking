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
import socketio

mode = 0
dt = 0
lat = 0
lon = 0
sats = 0
odo_VL = 0.1
odo_VR = 0.1
cond = "trouble: init. USB"

def send_error(cond,sio):
    maps_obj = {
        "latitude" : 0,
        "longitude" : 0,
        "heading" : 0,
        "satelite" : 0,
        "status" : cond
    }
    maps_json = json.dumps(maps_obj)
    sio.emit("gps",maps_json)

def main():
    global mode, dt, lat, lon, sats, odo_VL, odo_VR, cond
    sio = socketio.Client()
    sio.connect("http://localhost:3000")

    time_prev = time.time()
    odom=serial.Serial("/dev/ttyACM0",9600,timeout=1)
    odom.baudrate=9600
    gps=serial.Serial("/dev/ttyUSB0",9600,timeout=1)
    gps.baudrate=9600

    odom.reset_input_buffer()
    gps.reset_input_buffer()

    init = True
    while init: # initial measurement
        try:
            cond = "trouble: init. GPS"
            line_gps=gps.readline().decode('utf-8', errors='replace').strip()
            if '$GNGGA' in line_gps:
                msg = pynmea2.parse(line_gps)
                lat = msg.latitude
                lon = msg.longitude
                sats = msg.num_sats

                cond = "trouble: init. odometry"
                line_odom=odom.readline().decode('utf-8').split(',')
                parsed = [x.rstrip() for x in line_odom]
                if len(parsed) > 2:
                    odo_VL = float(parsed[0])
                    odo_VR = float(parsed[1])
                    mode = float(parsed[2])
                    init = False
        except:
            print(cond)
            send_error(cond,sio)
            pass

    while True: # Run measurement forever
        cond = "program running"
        try:
            line_odom=odom.readline().decode('utf-8').split(',')
            parsed = [x.rstrip() for x in line_odom]
            if len(parsed) > 2:
                odo_VL = float(parsed[0])
                odo_VR = float(parsed[1])
                mode = float(parsed[2])
                #print(parsed)
        except:
            cond = "no odometry meas."
            print(cond)
            pass

        try:
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
        except:
            cond = "no GPS meas."
            print(cond)
            pass

        time_now = time.time()
        dt = time_now-time_prev
        time_prev = time_now

        # maps = [latitude, longitude, heading]
        maps = ekf.filtering(mode,dt,lat,lon,odo_VL,odo_VR)
        maps_obj = {
            "latitude" : maps[0],
            "longitude" : maps[1],
            "heading" : maps[2],
            "satelite" : sats,
            "status" : cond
        }
        maps_json = json.dumps(maps_obj)
        print(maps_json)
        sio.emit("gps",maps_json)
        print(cond)
        
        
if __name__ == "__main__":
    main()