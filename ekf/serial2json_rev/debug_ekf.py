import pynmea2 #library for parsing GPS NMEA format
import time #library for time
import ekf
import numpy as np

mode = 1
dt = 1
lat = -6.914564
lon = 107.609810
odo_VL = -3
odo_VR = 4


# maps = [latitude, longitude, heading]
maps = ekf.filtering(mode,dt,lat,lon,odo_VL,odo_VR)

print(maps)