"""
#!/usr/bin/env python
#title           :SE100.py
#description     :Python Script to Read GPS Data from Radiolink-SE100 GPS Module Using Raspberry Pi (Both Geodetic-Coordinate and Heading)
#author          :Achmad Syahrul Irwansyah
#date            :2023/03/31
#version         :0.2
#usage           :Python
#notes           : - original file from read_HMC5983.py and gps_serial.py (combined)
                   - need to add database connection code
#python_version  :3.8
#==============================================================================
"""

from smbus import SMBus #library for i2c protocol in python
import time #library for time utilities
import math #library for math (trigonometry) utilities
import serial #library for serial communication
import pynmea2 #library for parsing GPS NMEA format
import datetime #library for date & time
import socketio
import json


sio = socketio.Client()
sio.connect("http://localhost:3000")


# Address of HMC5983
ADDRESS = 0x1E

#define register list from datasheet
CONRA = 0x00 #Configuration Register A
CONRB = 0x01 #Configuration Register B
MODREG = 0x02 #Mode Register
DOXMSB = 0x03 #Data Output X MSB Register
DOXLSB = 0x04 #Data Output X LSB Register
DOZMSB = 0x05 #Data Output Z MSB Register
DOZLSB = 0x06 #Data Output Z LSB Register
DOYMSB = 0x07 #Data Output Y MSB Register
DOYLSB = 0x08 #Data Output Y LSB Register
STAREG = 0x09 #Status Register
IDREGA = 0x0A #Identification Register A
IDREGB = 0x0B #Identification Register B
IDREGC = 0x0C #Identification Register C
TEMPMSB = 0x31 #Temperature Output MSB Register
TEMPLSB = 0x32 #Temperature Output LSB Register

# Create new I2C class for HMC5983 (Magnetometer)
HMC5983 = SMBus(1) # Use number that listed in 'ls /dev/i2*' to list all the i2c devices available

# Serial communication
try:
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=5) # Check the serial COM that is connected to SE100
    print("Connected to Serial COM")
except:
    print("Can't Connect to Serial COM")

# GPS Variables
la = None #longitude
lo = None #latitude
sats = None #number of satellites
hdop = None #horizontal dillution of precision
heading = None #orientation heading
timer = None #timestamp

# Function to read GPS
def read_gps():
    global la, lo, sats, hdop
    try:
        # Decode the data from GPS SE100 NMEA serial communication
        line = ser.readline().decode('utf-8', errors='replace')
        line = line.strip()
        #print(line)
        
        # Select the $GNGGA only
        if '$GNGGA' in line:
            #print(line)
                
            # Parse the data by using pynmea library
            msg = pynmea2.parse(line)
            
            # Variable
            la = msg.latitude
            lo = msg.longitude
            sats = msg.num_sats
            hdop = msg.horizontal_dil
    except:
        # Disconnected
        print("GPS NMEA SE100 DATA is not sent")
        print("================================")
        #print("")
        time.sleep(1)
        pass

# Funtion to warp value
def confit(value):
    if value > 32768:
        return value - 65535
    else:
        return value

# Funtion to read Magnetometer
def read_imu():
    global heading

    # Configuration setting
    HMC5983.write_byte_data(ADDRESS, MODREG, 0x01) #Mode Register (00000001): Single-Measurement Mode (default)
    HMC5983.write_byte_data(ADDRESS, CONRA, 0x10) #Conf. Reg. A (00010000): Temperature sensor is disable, 1 sample average per measurement, 15 Hz data output, Normal measurement configuration
    HMC5983.write_byte_data(ADDRESS, CONRB, 0x20) #Conf. Reg. B (00100000): Sensor Field Range of ±1.3 Ga with 0.92 mG/LSb digital resolution (default)
    
    # Retrieve measurements byte data
    block = HMC5983.read_i2c_block_data(ADDRESS, DOXMSB, 32)
    x_msb = block[0]
    x_lsb = block[1]
    z_msb = block[2]
    z_lsb = block[3]
    y_msb = block[4]
    y_lsb = block[5]

    # Decode the byte data
    x_raw = x_msb * 256 + x_lsb
    z_raw = z_msb * 256 + z_lsb
    y_raw = y_msb * 256 + y_lsb

    # Warp the value
    x = confit(x_raw)
    z = confit(z_raw)
    y = confit(y_raw)

    # Heading calculation
    heading = math.atan2(y, x) * 180 / math.pi

if __name__ == "__main__":
    try:
        print("Timestamp, Latitude, Longitude, Heading, Number_of_Satellites, HDOP")
        
        # Timer
        timer = datetime.datetime.now()

        while True:
            read_gps()
            read_imu()
            print(timer.strftime("%Y-%m-%d %H:%M:%S"),", ",la,", ",lo,", ",round(heading,2),", ",sats,", ",hdop)
            gpsdat = {"latitude" : la, "longitude" : lo, "heading" : heading, "satelite" : sats, "hdop" : hdop}
            gps2js = json.dumps(gpsdat)
            sio.emit("gps",gps2js)


            time.sleep(0.5)
    except:
        pass
