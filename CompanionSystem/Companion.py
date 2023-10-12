# Import Libraries / Modules
import os                                                       # Operating System Interfaces
from picamera import PiCamera                                   # Raspberry Pi Camera
from picamera import Color                                      # Raspberry Pi Camera
import threading                                                # Thread-Based Parallelism
import VL53L1X                                                  # Time-of-Flight sensor
import time                                                     # Time access and conversion module
from datetime import datetime                                   # Date and time module
from pymavlink import mavutil                                   # Communication for Ardupilot Mavlink Protocol

import TMP117
import SHTC3
import CCS811
import self_bme280
import Micropressure

startTime=time.time()                                           # Use to mark a starting time for code, used for timing and data recording.

class sensorBus (threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.checkConn()                                            # Attempt to initialize all possible sensors
        self.dataFull=[]                                            # Initialize data array
        self.header=[]                                              # initialize header array
        self.makeHeader()                                           # Create header
        
    # Create header for recorded data based on what sensors passed initialization
    def makeHeader(self):
        self.header.append("Time (s)")
        if self.tmp._running:
            self.header.append("TMP117 Temp (C)")
        if self.shtc3._running:
            self.header.append("SHTC3 Temp (C)")
            self.header.append("SHTC Humidity (%)")
        if self.micro._running:
            self.header.append("Micro Pressure (psi)")
        if self.bme._running:
            self.header.append("BME280 Temp (C)")
            self.header.append("BME280 Press. (Pa)")
            self.header.append("BME280 Humidity (%)")
            self.header.append("BME280 Altitude (m)")
        if self.ccs._running:
            self.header.append("CCS811 tVOC (ppb)")
            self.header.append("CCS811 CO2 (ppm)")
            
    # Create Data array to be recorded based on what sensors passed initialization        
    def fillData(self):
        self.dataFull=[]
        self.dataFull.append((self.timer1-startTime))
        if self.tmp._running:
            self.dataFull.append(self.tmp.temp_c)
        if self.shtc3._running:
            self.dataFull.append(self.shtc3.temp)
            self.dataFull.append(self.shtc3.humidity)
        if self.micro._running:
            self.dataFull.append(self.micro.Pressure)
        if self.bme._running:
            self.dataFull.append(self.bme.temperature)
            self.dataFull.append(self.bme.pressure)
            self.dataFull.append(self.bme.humidity)
            self.dataFull.append(self.bme.altitude)
        if self.ccs._running:
            self.dataFull.append(self.ccs.eTVOC)
            self.dataFull.append(self.ccs.eco2)
            
    # What will run inside the thread        
    def run(self):
        while True:
            self.timer1=time.time()                                 # Mark current time
            
            # Read latest available data from all sensors
            self.micro.read_pressure()
            self.shtc3.measure()
            self.tmp.tempReading()
            self.bme.get_bme280_data()
            self.ccs.read_gas()
            
            # For timing purposes
            self.timer2=time.time()
            self.timeTotal=self.timer2-self.timer1
            print("Time to collect all sensors: ",self.timeTotal)
            
            self.fillData()                                        # Create data array from newly recorded sensors
           
            time.sleep(.05)                                         # Sleep 50ms
    
    # Attempt to initialize all possible sensor
    def checkConn(self):
        self.tmp=TMP117.TMP117()
        self.shtc3=SHTC3.SHTC3()
        self.bme=self_bme280.bme280_sensor()
        self.ccs=CCS811.CCS811()
        self.micro=Micropressure.microPressure()

       
class dataRecordBus (threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.name_file = datetime.now().strftime('%Y%m%d_%H%M%S')   # Create file name
        os.mkdir("/home/pi/Documents/logs/" + self.name_file)       # Create log directory
        self.log_file = self.name_file + "/" + self.name_file + ".txt" # Create log file
        self.video_file = "/home/pi/Documents/logs/" + self.name_file + "/" + self.name_file + ".h264" # Create video file
        
    # Thread main code
    def run(self):
        self.log_data_file(sensor.header)                           # Log header data into log file
        while True:
            self.log_data_file(sensor.dataFull)                     # Log current sensor data
            time.sleep(.05)                                         # Sleep 50ms
    # Save data in log text file        
    def log_data_file(self, data):
        file = open(r"/home/pi/Documents/logs/" + self.log_file, "a") # open log fil
        for L in range(len(data)): # Append data
            if L == len(data) - 1:
                file.write(str(data[L]) + "\r\n")
            else:
                file.write(str(data[L]) + ",")
        file.close() # Close file

sensor=sensorBus()
record=dataRecordBus()
sensor.start()
record.start()

