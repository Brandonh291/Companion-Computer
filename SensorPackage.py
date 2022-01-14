#######################################################
# Current Date of Version: 1/14/2022
#######################################################
#The service's name is sensorPackage.service

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
import time
from smbus2 import SMBus
from enum import Enum
import qwiic_vl53l1x
import sys
from datetime import datetime
from math import log10, floor
from pymavlink import mavutil
########################################################
# TMP117 Registers
tmp117_addr = 0x48
tmp117_reg_temp = 0x00
tmp117_reg_config = 0x01
########################################################
# Name for Log File, Gives Date and Time
log_file=datetime.now().strftime('%Y%m%d_%H%M%S')+".txt"
##############################################
# Dicitionaries for All Sensors, You just add a sensor here and inside the sensor_list array
tmp117_dict={
    "Name":"TMP117 Temperature Sensor",
    "Address":0x48,
    "Logged Data":"Temperature (C)"
    }
vl53l1x_dict={
    "Name":"VL53L1X Distance Sensor",
    "Address":0x29,
    "Logged Data":"Distance (mm)"
    }
##############################################
sensor_list=[vl53l1x_dict,tmp117_dict] # This list should be from lowest address to highest so the data markers are correct
logged_data=["time(s"] # Array for Data that will be logged
activeSensors=[] # Array of active sensors
#nav_time=0
#nav_lat=0
#nav_long=0
#nav_alt=0
##############################################
# Calculate the 2's complement of a number
def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val
##############################################
# Compare Active I2C Addresses with Sensor List for matches
def print_active_sensors():
    for x in activeSensors:
        for y in sensor_list:
            if x==y["Address"]:
                print(y["Name"])
                logged_data.append(y["Logged Data"])
##############################################
# Check for active I2C Addresses and Sensors
def check_Available_Sensors():
    print("Sensor Addresses Available: ")
    bus=SMBus(1)
    for address in range(128):
    
        try:
            val=bus.read_byte_data(address,0x00)
            print("Good: ",hex(address))
            activeSensors.append(address)
            
        except:
            #print("Bad: ",hex(address))
            pass
    bus.close()
    print("\n")
    print("Active Sensors: ")
    print_active_sensors()
    print("\n")
#     print("Data Being Logged: ")
#     print(logged_data)
#     print("\n")
##############################################
# Read temperature registers and calculate Celsius
def read_temp():
    # Initialize I2C (SMBus)
    bus = SMBus(1)

    # Read the CONFIG register (2 bytes)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    #print("Old CONFIG:", hex(val[0]),hex(val[1]))

    # Set to 4 Hz sampling (CR1, CR0 = 0b10)
    val[1] = val[1] & 0b00111111
    val[1] = val[1] | (0b10 << 6)

    # Write 4 Hz sampling back to CONFIG
    bus.write_i2c_block_data(tmp117_addr, tmp117_reg_config, val)

    # Read CONFIG to verify that we changed it
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    #print("New CONFIG:", hex(val[0]),hex(val[1]))
    # Print out temperature every second
    # Read temperature registers
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_temp, 2)
    # NOTE: val[0] = MSB byte 1, val [1] = LSB byte 2
    #print ("!shifted val[0] = ", bin(val[0]), "val[1] = ", bin(val[1]))
    #print(val[0])
    #print(val[1])
    temp_c = (val[0] << 8) | (val[1] >> 0)
    #print (" shifted val[0] = ", bin(val[0] << 4), "val[1] = ", bin(val[1] >> 4))
    #print (bin(temp_c))

    # Convert to 2s complement (temperatures can be negative)
    #print(bin(temp_c))
    temp_c = twos_comp(temp_c, 16)
    #print(bin(temp_c))
    #print((temp_c))
    # Convert registers value to temperature (C)
    temp_c = temp_c * 0.0078125
    bus.close()
    return temp_c
##############################################
def to_die():
    return 0
##############################################
# Measures Distance sensor and returns value in millimeters
def measure_distance():
    #rangeSensor=qwiic_vl53l1x.QwiicVL53L1X()
    #rangeSensor.sensor_init()
    rangeSensor.start_ranging()
    time.sleep(0.005)
    distance=rangeSensor.get_distance()
    time.sleep(0.005)
    rangeSensor.stop_ranging()
    return distance
##############################################
# Log Iteration of Data Measurements
def log_data_file(data):
    #log data to an file as an append
    file=open(r"/home/pi/Documents/logs/"+log_file,"a")
    for L in range(len(data)):
        if L == len(data)-1:
            file.write(str(data[L]) + "\r\n")
        else:
            file.write(str(data[L]) + ",")
    file.close()
##############################################
# Initialize Data Log with Available Data recorded
def initialize_log_file():
    #log data to an file as an append
    file=open(r"/home/pi/Documents/logs/"+log_file,"a")
    for L in logged_data:
        if L == logged_data[len(logged_data)-1]:
            file.write(str(L) + "\r\n")
        else:
            file.write(str(L) + ",")
    file.close()
#########################################
# So here I want to select what Mavlink Data to bring in to the Data Log
def initialize_mavlink_log():
    global master
    master=mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
    print("Waiting for Heartbeat")
    master.wait_heartbeat()
    print("Heartbeat Received")
    master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL, 1,0)
    time.sleep(0.1)
    master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,1e6/1,2,0,0,0,0)
    time.sleep(0.1)
    master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_GPS_STATUS,1e6/1,2,0,0,0,0)
    time.sleep(0.1)
    #master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1,1)
    #master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1,1)
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
test_mode=1 # This is whether we ignore whether the system is armed or not and performs the data logging
check_once=0
bad_data=0
#########################################
# Main Code
while True:
    test_time=0
    distanceInitialize = 0
    check_global_pos_int=0
    check_Available_Sensors()
    initialize_mavlink_log()

    #print("Are we armmed? :" + str(master.motors_armed()))
    print("Data Being Logged: ")
    print(logged_data)
    print("\n")

    initialize_log_file()

    start_time=time.time()


    while master.motors_armed()==128 or test_mode==1:
        #print("Are we armmed? :" + str(master.motors_armed()))
        data=[]

        data.append(round(time.time()-start_time,3))
        test_time=time.time()
        itr_time=time.time()

        for i in activeSensors:
            sense_time=time.time()
            
            if i == 0x29:
                try:
                    if distanceInitialize==1:
                        distance=measure_distance()
                        data.append(distance)
                    else:
                        global rangeSensor
                        rangeSensor=qwiic_vl53l1x.QwiicVL53L1X()
                        rangeSensor.sensor_init()
                        distanceInitialize=1
                except:
                    print("Sensor Error Has Occured, Ignoring Distance Readings Until Sensor is Fixed")
                    data.append("")
            elif i == 0x48:
                try:
                    temperature = read_temp()
                    data.append(temperature)
                except:
                    print("Sensor Error Has Occured, Ignoring Temperature Readings Until Sensor is Fixed")
                    data.append("")
            elif i == 0x50:
                if test_mode == 1: 
                    print("Performing Death")
                rangeItUp=to_die()
                #data.append(rangeItUp)
                
        #print(temperature, "C")
                #["Navio2_Time_ms","Navio2_latitude","Navio2_longitude","Navio2_Relative_Altitude"])
    #     master.mav.command_long_send(master.target_system,master.target_component,
    #                                   mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,0,mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,0,0,0,0,0,0)
    #     try:
    #         print(master.recv_match().to_dict())
    #     except:
    #         pass
        test_time=time.time()
        try:
             
            #master.mav.command_long_send(master.target_system, master.target_component,
                                    #mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 33, 0, 0, 0, 0, 0, 0)
                
            #master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,33, 0, 0, 0, 0, 0, 0)
            time.sleep(.25)
            msg=master.recv_match().to_dict()
            try:
                if msg['mavpackettype']=='BAD_DATA':
                    bad_data=bad_data+1
                    print("Consecutive Bad Data: ",bad_data)
                    print(msg)
            except:
                pass
            if check_once==0:
                        base_time=msg['time_boot_ms']
                        check_once=1
            try:
                over_time=msg['time_boot_ms']
            except:
                pass
            #if test_mode == 1:
                #print(msg)
            print(msg['mavpackettype'])
            try:
                print((over_time-base_time)/1000)
                print(data[0]-(over_time-base_time)/1000)
                if (data[0]-(over_time-base_time)/1000) > 5:
                            overtime=1
                            while overtime==1:
                                
                                msg=master.recv_match().to_dict()
                                try:
                                    over_time=msg['time_boot_ms']
                                except:
                                    pass
                                #print(data[0]-(over_time-base_time)/1000)
                                if (data[0]-(over_time-base_time)/1000) < 1:
                                    overtime=0
            except:
                pass
            
            if msg['mavpackettype'] == 'NAV_CONTROLLER_OUTPUT':
                try:
                    print(msg['wp_dist'])
                except:
                    print("Time Failed to Load")
            
            if msg['mavpackettype'] == 'GLOBAL_POSITION_INT':
                try:
                    bad_data=0
                    nav_time=msg['time_boot_ms']
                    nav_lat = msg['lat']
                    nav_long = msg['lon']
                    nav_rel_alt = msg['relative_alt']
                    nav_alt = msg['alt']
                    nav_vx = msg['vx']
                    nav_vy = msg['vy']
                    nav_vz = msg['vz']
                    nav_hdg = msg['hdg']
                    #print(msg['time_boot_ms'])
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
            #print("Exception in Mavlink")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            data.append("")
            
            
        try:
            print(data)
        except:
            pass
        
        test_time=time.time()
        log_data_file(data)
        #time.sleep(0.1)
        
            
    while master.motors_armed()==0:
        # Since we are disarmed, we are going to reset the active sensors and create a new log file name that is just going to be the current time
        # That way when we arm up again it will make a new file.
        log_file=datetime.now().strftime('%Y%m%d_%H%M%S')+".txt"
        logged_data=["time(s"] # Array for Data that will be logged
        activeSensors=[] # Array of active sensors
        #print("Are we armmed? :" + str(master.motors_armed()))
        
        #master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,33, 0, 0, 0, 0, 0, 0)
        time.sleep(.1)
        try:
            msg=master.recv_match().to_dict()
            print(msg['mavpackettype'])
        except:
            pass
        #time.sleep(.5)
