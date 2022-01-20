# Libraries
import time
#from threading import Thread
from smbus2 import SMBus
from enum import Enum
import qwiic_vl53l1x
import sys
from datetime import datetime
from math import log10, floor
from pymavlink import mavutil
ccs811_addr=0x5B
bme280_addr=0x77
global eco2
from bme280 import BME280
def read_distance():
    bus=SMBus(4)
    #send  send 0x40 to SYSTEM_MODE_START register  byte
    # to stop send 0x00 to SYSTEM_MODE_START register 1 byte
    system_mode_start=0x0087
    #bus.write_byte(0x52,
    
def read_humidity_press():
    for i in range(30):
        bus=SMBus(4)
        bme280 = BME280(i2c_dev=bus,i2c_addr=0x77)
        temperature = bme280.get_temperature()
        pressure = bme280.get_pressure()
        humidity = bme280.get_humidity()
        altitude=bme280.get_altitude()
        print("Altitude: ",altitude)
        print('{:05.2f}*C {:05.2f}hPa {:05.2f}%'.format(temperature, pressure, humidity))
        #mySensor1.set_environmental_data(humidity, temperature)
        time.sleep(1)
def read_gas():
    global eco2
    global eTVOC
    # Will need to wait 20 minutes for decent readings
    bus = SMBus(4)
    
    # Done at Startup, exit boot
    bus.write_byte(0x5B,0xF4)
    
    #Set Mode
    bus.write_byte(0x5B,0x01)
    bus.write_i2c_block_data(0x5B,0x01,[0b00010000])
    for i in range(10):
        # Go to Data
        #Ignore first 3, they are calibrations
        bus.write_byte(0x5B,0x02)
        val=bus.read_i2c_block_data(0x5b,0x02,4)
        eco2=((val[0]<<8)|(val[1]))
        eTVOC=((val[2]<<8)|(val[3]))
        print("eCO2: "+ str(eco2) + " ppm")
        print("eTVOC: "+ str(eTVOC) + "ppb")
        time.sleep(1)
    
    
def print_gas():
    time.sleep(2)
    print(eco2)
    
    
def read_temp():
    # Initialize I2C (SMBus)
    bus = SMBus(1)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    val[1] = val[1] & 0b00111111
    val[1] = val[1] | (0b10 << 6)
    bus.write_i2c_block_data(tmp117_addr, tmp117_reg_config, val)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_temp, 2)
    temp_c = (val[0] << 8) | (val[1] >> 0)
    temp_c = twos_comp(temp_c, 16)
    # Convert registers value to temperature (C)
    temp_c = temp_c * 0.0078125
    bus.close()
    return temp_c

#threadIt=Thread(target=print_gas)
#threadIt.start()

read_gas()
#hreadIt.terminate()
read_humidity_press()