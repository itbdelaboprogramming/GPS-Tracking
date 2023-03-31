#!/usr/bin/env python

from smbus import SMBus
import time
import math


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

# Create new I2C class for HMC5983
HMC5983 = SMBus(1)
# Use number that listed in 'ls /dev/i2*' to list all the i2c devices available

def confit(value):
    if value > 32768:
        return value - 65535
    else:
        return value

while True:
    HMC5983.write_byte_data(ADDRESS, MODREG, 0x01)
    HMC5983.write_byte_data(ADDRESS, CONRA, 0x10)
    HMC5983.write_byte_data(ADDRESS, CONRB, 0x20)
    block = HMC5983.read_i2c_block_data(ADDRESS, DOXMSB, 32)

    x_msb = block[0]
    x_lsb = block[1]
    z_msb = block[2]
    z_lsb = block[3]
    y_msb = block[4]
    y_lsb = block[5]

    x_raw = x_msb * 256 + x_lsb
    z_raw = z_msb * 256 + z_lsb
    y_raw = y_msb * 256 + y_lsb

    x = confit(x_raw)
    z = confit(z_raw)
    y = confit(y_raw)

    heading = math.atan2(y, x) * 180 / math.pi
    print(round(heading, 2))

    time.sleep(0.5)