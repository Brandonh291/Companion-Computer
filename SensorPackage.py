#######################################################
# Current Date of Version: 1/18/2022
#######################################################
# The service's name is sensorPackage.service

# To START use: "sudo systemctl enable sensorPackage.service"
# To STOP  use: "sudo systemctl stop sensorPackage.service"
#######################################################
# To commit this back to the git page
# git add SensorPackage.py
# git commit -m "COMMENT"
# git push origin main
#######################################################
# How to update?
# git pull origin main
#######################################################
# Libraries
from threading import Thread
import pip
import time
from smbus2 import SMBus  # Install in terminal via: "sudo pip install smbus2"
import sys
from enum import Enum
from datetime import datetime
from math import log10, floor
from bme280 import BME280 # for BME280: "sudo pip install pimoroni-bme280"
import qwiic_ccs811 # for CCS811: "sudo pip install sparkfun-qwiic-ccs811"
# for Github: "sudo apt-get install git"
from pymavlink import mavutil  # Install in terminal via: "sudo python -m pip install --upgrade pymavlink"
import qwiic_vl53l1x  # Install in terminal via: "sudo pip install sparkfun-qwiic-vl53l1x"

########################################################
# TMP117 Registers
tmp117_addr = 0x48
tmp117_reg_temp = 0x00
tmp117_reg_config = 0x01
########################################################
# Name for Log File, Gives Date and Time
log_file = datetime.now().strftime('%Y%m%d_%H%M%S') + ".txt"
##############################################
logged_data = ["time(s"]  # Array for Data that will be logged
activeSensors = []  # Array of active sensors
data=[]
########################################################
# Threading Testing
bus = SMBus(1)
class bme280_ccs811_sensor:
    def __init__(self):
        #  # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        # might need to delcare these sensors global
        try:
            global bme280
            global ccs811Sense
            ccs811Sense = qwiic_ccs811.QwiicCcs811()
            bme280 = BME280(i2c_dev=bus,i2c_addr=0x77)
            ccs811Sense.begin()
            logged_data.append("BME280 Temperature (C)")
            logged_data.append("BME280 Pressure (hPa)")
            logged_data.append("BME280 Humidity (%)")
            logged_data.append("BME280 Altitude (ft)")
            logged_data.append("CCS811 CO2 (ppm)")
            logged_data.append("CCS811 tVOC (ppb)")
            self._running = True
        except:
            print("BME280 and CCS811 Sensor not Detected")
            self._running = False
            return
        
        
        
    def get_ccs811_bme280_data(self):
        try:
            if self._running:
                global data
                temperature = bme280.get_temperature()
                data.append(temperature)
                pressure = bme280.get_pressure()
                data.append(pressure)
                humidity = bme280.get_humidity()
                data.append(humidity)
                altitude=bme280.get_altitude()
                data.append(altitude)
                ccs811Sense.set_environmental_data(humidity, temperature)
                if ccs811Sense.data_available():
                    ccs811Sense.read_algorithm_results()
                    data.append(ccs811Sense.CO2)
                    data.append(ccs811Sense.TVOC)
                else:
                    append_Blanks(2)
        except:
            if self._running:
                print("Connection to Environmental Sensor Lost")
                append_Blanks(6)
                
def append_Blanks(num_blank):
    global data
    for i in range(num_blank):
        data.append("")
class VL53l1x_distance_sensor:
    def __init__(self):
          # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        try:
            global rangeSensor
            bus.read_byte_data(0x29, 0x00)
            print("Received a Response")
            logged_data.append("VL53L1X Distance (mm)")
            rangeSensor = qwiic_vl53l1x.QwiicVL53L1X()
            rangeSensor.sensor_init()
            self._running = True
        except:
            print("VL53L1X Sensor not Detected")
            self._running = False
            return
        

    def measure_distance(self):
        try:
            if self._running:
                global data
                rangeSensor.start_ranging()
                time.sleep(0.005)
                distance = rangeSensor.get_distance()
                time.sleep(0.005)
                rangeSensor.stop_ranging()
                data.append(distance)
                return distance
        except:
            if self._running:
                print("Connection to VL53L1X Lost")
                append_Blanks(1)

class Tmp117_Sensor:
    def __init__(self):
          # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        try:
            bus.read_byte_data(tmp117_addr, 0x00)
            print("Received a Response")
            logged_data.append("TMP 117 temperature (C)")
            self._running = True
        except:
            print("TMP117 Sensor not Detected")
            self._running = False
            return
        

    def terminate(self):
        self._running = False

    # Calculate the 2's complement of a number
    def twos_comp(self,val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def tempReading(self):
        try:
            if self._running:
                # Initialize I2C (SMBus)
                
                global data
                # Read the CONFIG register (2 bytes)
                val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
                # print("Old CONFIG:", hex(val[0]),hex(val[1]))

                # Set to 4 Hz sampling (CR1, CR0 = 0b10)
                val[1] = val[1] & 0b00111111
                val[1] = val[1] | (0b10 << 6)

                # Write 4 Hz sampling back to CONFIG
                bus.write_i2c_block_data(tmp117_addr, tmp117_reg_config, val)

                # Read CONFIG to verify that we changed it
                val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
                # Print out temperature every second
                # Read temperature registers
                val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_temp, 2)
                temp_c = (val[0] << 8) | (val[1] >> 0)
                temp_c = self.twos_comp(temp_c, 16)
                # Convert registers value to temperature (C)
                temp_c = temp_c * 0.0078125
                
                data.append(temp_c)
                return temp_c
        except:
            if self._running:
                print("Connection to TMP117 Lost")
                append_Blanks(1)
            

##############################################

##############################################
# Log Iteration of Data Measurements
def log_data_file(data):
    # log data to an file as an append
    file = open(r"/home/pi/Documents/logs/" + log_file, "a")
    for L in range(len(data)):
        if L == len(data) - 1:
            file.write(str(data[L]) + "\r\n")
        else:
            file.write(str(data[L]) + ",")
    file.close()


##############################################
# Initialize Data Log with Available Data recorded
def initialize_log_file():
    # log data to an file as an append
    file = open(r"/home/pi/Documents/logs/" + log_file, "a")
    for L in logged_data:
        if L == logged_data[len(logged_data) - 1]:
            file.write(str(L) + "\r\n")
        else:
            file.write(str(L) + ",")
    file.close()


#########################################
# So here I want to select what Mavlink Data to bring in to the Data Log
def initialize_mavlink():
    global master
    master = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
    print("Waiting for Heartbeat")
    master.wait_heartbeat()
    print("Heartbeat Received")
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 0)
    time.sleep(0.1)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1e6 / 1, 2, 0, 0, 0, 0)
    time.sleep(0.1)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_GPS_STATUS, 1e6 / 1, 2, 0, 0, 0, 0)
    time.sleep(0.1)
    # master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1,1)
    # master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1,1)

def initialize_mavlink_log():
    global logged_data
    logged_data.append("Navio2_Time (GLOBAL_POSITION_INT, ms)")
    logged_data.append("Navio2_latitude (GLOBAL_POSITION_INT, degE7)")
    logged_data.append("Navio2_longitude (GLOBAL_POSITION_INT, degE7)")
    logged_data.append("Navio2_Relative_Altitude (GLOBAL_POSITION_INT, mm)")
    logged_data.append("Navio2_Altitude (GLOBAL_POSITION_INT, mm)")
    logged_data.append("Navio2_Ground_X_Speed (GLOBAL_POSITION_INT, cm/s)")
    logged_data.append("Navio2_Ground_Y_Speed (GLOBAL_POSITION_INT, cm/s)")
    logged_data.append("Navio2_Ground_Z_Speed (GLOBAL_POSITION_INT, cm/s)")
    logged_data.append("Navio2_Heading (GLOBAL_POSITION_INT, cdeg)")
# def read_mavlink_data(search):
#     msg=master.recv_match().to_dict()
#     if msg['mavpackettype'] == 'GLOBAL_POSITION_INT':
#         print("Altitude: %s m " %(msg['relative_alt']/1000))
#         altitude=msg['relative_alt']/1000

#########################################
# Adjustable Variables
test_mode = 0  # This is whether we ignore whether the system is armed or not and performs the data logging
check_once = 0
bad_data = 0
#########################################

# tempSense = Tmp117_Sensor()
# distSense = VL53l1x_distance_sensor()
# envSense = bme280_ccs811_sensor()
# print(distSense.measure_distance())
# print(tempSense.tempReading())

heartBeatArmed=0
initialize_items=0
initialize_mavlink()
# Main Code
while True:

    start_time = time.time()
    while heartBeatArmed != 129:
        # Since we are disarmed, we are going to reset the active sensors and create a new log file name that is just going to be the current time
        # That way when we arm up again it will make a new file.
        log_file = datetime.now().strftime('%Y%m%d_%H%M%S') + ".txt"
        logged_data = ["time(s"]  # Array for Data that will be logged
        initialize_items=0
        # master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,33, 0, 0, 0, 0, 0, 0)
        time.sleep(.1)
        try:
            msg = master.recv_match().to_dict()
            print(msg['mavpackettype'])
            if msg['mavpackettype']== 'HEARTBEAT':
                heartBeatArmed=msg['base_mode']
                #print(heartBeatArmed)
        except:
            pass
        #time.sleep(.5)
        
    while heartBeatArmed==129 or test_mode == 1:
        if initialize_items==0: 
            distanceInitialize = 0
            check_global_pos_int = 0
            tempSense = Tmp117_Sensor()
            distSense = VL53l1x_distance_sensor()
            envSense = bme280_ccs811_sensor()
            initialize_mavlink_log()
            print("Data Being Logged: ")
            print(logged_data)
            print("\n")
            initialize_log_file()
            initialize_items=1
        
        
        data = []
        data.append(round(time.time() - start_time, 3))
        # Get VL53L1X Data
        distance = distSense.measure_distance()
        # Get TMP117 Data
        temperature = tempSense.tempReading()
        # Get BME280 and CCS811 data
        envSense.get_ccs811_bme280_data()
        
        try:

            # master.mav.command_long_send(master.target_system, master.target_component,
            # mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 33, 0, 0, 0, 0, 0, 0)

            # master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,33, 0, 0, 0, 0, 0, 0)
            time.sleep(.25)
            msg = master.recv_match().to_dict()
            try:
                if msg['mavpackettype'] == 'BAD_DATA':
                    bad_data = bad_data + 1
                    print("Consecutive Bad Data: ", bad_data)
                    print(msg)
            except:
                pass
            if check_once == 0:
                base_time = msg['time_boot_ms']
                check_once = 1
            try:
                over_time = msg['time_boot_ms']
            except:
                pass
            if test_mode == 1:
                print(msg['mavpackettype'])
            try:
                #print((over_time - base_time) / 1000)
                #print(data[0] - (over_time - base_time) / 1000)
                if (data[0] - (over_time - base_time) / 1000) > 5:
                    overtime = 1
                    while overtime == 1:

                        msg = master.recv_match().to_dict()
                        try:
                            over_time = msg['time_boot_ms']
                        except:
                            pass
                        # print(data[0]-(over_time-base_time)/1000)
                        if (data[0] - (over_time - base_time) / 1000) < 1:
                            overtime = 0
            except:
                pass

            if msg['mavpackettype'] == 'NAV_CONTROLLER_OUTPUT':
                try:
                    print(msg['wp_dist'])
                except:
                    print("Time Failed to Load")
                    
            if msg['mavpackettype']== 'HEARTBEAT':
                #print(msg)
                if msg['base_mode'] == 0:
                    heartBeatCount=heartBeatCount+1
                else:
                    heartBeatCount=0
                    
                if heartBeatCount>3:
                    heartBeatArmed=0
                #heartBeatArmed=msg['base_mode']
                #print(heartBeatCount)
                
            if msg['mavpackettype'] == 'GLOBAL_POSITION_INT':
                try:
                    bad_data = 0
                    nav_time = msg['time_boot_ms']
                    nav_lat = msg['lat']
                    nav_long = msg['lon']
                    nav_rel_alt = msg['relative_alt']
                    nav_alt = msg['alt']
                    nav_vx = msg['vx']
                    nav_vy = msg['vy']
                    nav_vz = msg['vz']
                    nav_hdg = msg['hdg']
                    # print(msg['time_boot_ms'])
                except:
                    print("Global Position Failed ")

            data.append(nav_time)
            data.append(nav_lat)
            data.append(nav_long)
            data.append(nav_rel_alt)
            data.append(nav_alt)
            data.append(nav_vx)
            data.append(nav_vy)
            data.append(nav_vz)
            data.append(nav_hdg)

        except:
            # print("Exception in Mavlink")
            append_Blanks(9)
        try:
            print(data)
        except:
            pass
        log_data_file(data)
        # time.sleep(0.1)
