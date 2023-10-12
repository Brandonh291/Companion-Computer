# Companion Computer Code

# By:       Brandon Hickey
# Date:     7/24/2023
# Version   2.0


# Import Libraries / Modules
import os                                                       # Operating System Interfaces
from picamera import PiCamera                                   # Raspberry Pi Camera
from picamera import Color                                      # Raspberry Pi Camera
import threading                                                # Thread-Based Parallelism
import VL53L1X                                                  # Time-of-Flight sensor
import time                                                     # Time access and conversion module
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
from datetime import datetime                                   # Date and time module
from bme280 import BME280                                       # BME280 environmental sensor
from pymavlink import mavutil                                   # Communication for Ardupilot Mavlink Protocol
import TMP117

# Initialize Bus and Time
bus = SMBus(4)                                                  # Use "1" for GPIO 2 & 3, use "4" for GPIO 23 & 24 
                                                                # Defined in your "/boot/config.txt" file, my selections are below.
                                                                # GPIO 23  (Pin16) = SDA,       GPIO 24  (Pin 18) = SCL
                                                                # GPIO 2   (Pin3)  = SDA,       GPIO 3   (Pin 5)  = SCL
startTime=time.time()                                           # Use to mark a starting time for code, used for timing and data recording.


# TMP117 Temperature Sensor
# Sparkfun Breakout Board
class TMP117:
    def __init__(self):
        # Attempt to initialize device
        try:                                    
        
            self.tmp117_addr = 0x48                             # Device Address 
            self.tmp117_reg_temp = 0x00                         # Temperature Register Address
            self.tmp117_reg_config = 0x01                       # Configuration Register Address
            
            self.dataHere = False                               # Initialize Flag for whether there is data to be read
            self.temp_c = 0                                     # Initialize data output......Removeable?
            val=[0,1]
            bus.read_byte_data(self.tmp117_addr, 0x00)          # Read the temperature register.


            # Configure Sensor 
            val[1] = 0b10111100                                 # LSB of Configuration Register
            val[0] = 0b00000000                                 # MSB of Configuration Register
            bus.write_i2c_block_data(self.tmp117_addr,self.tmp117_reg_config, val)          # Write the new configuration to the register.                

            self._running = True                                # Initialization succeeded
                                                                # System will take data from this sensor.
                                                        
            print("TMP117 Pass")                                # Let the user know the sensor passed initialization.
            
        # Failure to initialize device.
        except:
            self._running = False                               # Initialization failed, mark as non-functioning.
                                                                # System will not take data from this sensor.
                                                        
            print("TMP117 Fail")                                # Let the user know the sensor failed initialization.
            
    # Twos Complement Function 
    def twos_comp(self, val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val
        
    # Check if new data is available.
    def dataReady(self):
        val = bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_config, 2)         # Read the Configuration Register.
        
        if val[0] &0b100000:                                    # Check if Bit 13 of the configuration register is "1".
            self.dataHere = True                                # If Bit 13 == 1, New data is available.   
        else:
            self.dataHere = False                               # Else, new data is not available.
  
    # Read Data from Register
    def tempReading(self):
        if self._running == 1:                                    # Only reads data if system passed initialization.
            self.dataReady()                                    # Check if data is available to be read. 
            if self.dataHere:                                   # If data is available, read from temperature register.
                val = bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_temp, 2) # Read temperature data register and store the recieved 2 bytes.                    
                self.temp_c = (val[0] << 8) | (val[1] >> 0)     # Combine MSB and LSB bytes of the data.
                self.temp_c = self.twos_comp(self.temp_c, 16)   # Calculate Twos complement of combined data.

                self.temp_c = self.temp_c * 0.0078125           # Convert to celsius. 


# SHTC3 Sensor
# Sparkfun Breakout Board
class SHTC3:
    def __init__(self):
        # Attempt to initialize device
        try:
            self.addr = 0x70                                      # Device Address
            self.wakeupMSB = 0x35                                 # MSB of Wakeup Command
            self.wakeupLSB = 0x17                                 # LSB of Wakeup Command
            self.sleepMSB = 0xb0                                  # MSB of Sleep Command
            self.sleepLSB = 0x98                                  # LSB of Sleep Command

            msg=i2c_msg.write(0x70,[0x35,0x17])                 # Write Wakeup Command
            bus.i2c_rdwr(msg)                                   # Send Wakeup Command
            msg=i2c_msg.write(0x70,[0b01011100,0b00100100])     # Write Measure Command
            bus.i2c_rdwr(msg)                                   # Send Measure Command
            msg=i2c_msg.read(0x70,6)                            # Write Read Command
            bus.i2c_rdwr(msg)                                   # Send Read Command
            
            self._running = True                                # Initialization Succeeded
                                                                # System will take data from sensor.
            print("SHTC3 Pass")                                 # Let user know the sensor passed initialization.
            
        # Failure to initialize device.
        except:
            self._running = False                               # Initialization Failed.
                                                                # System will not take data from sensor.
            print("SHTC3 Fail")                                 # Let user know the sensor failed initialization.
            
    # Read Temperature and Humidity Data    
    def measure(self):
        if self._running:
            msg = i2c_msg.write(0x70,[0x35,0x17])                 # Write Wakeup Command
            bus.i2c_rdwr(msg)                                   # Send Wakeup Command
            msg = i2c_msg.write(0x70,[0b01011100,0b00100100])     # Write Measure Command
            bus.i2c_rdwr(msg)                                   # Send Measure Command
            msg = i2c_msg.read(0x70,6)                            # Write Read Command
            bus.i2c_rdwr(msg)                                   # Send Read Command
            
            data = list(msg)                                      # Load data from command into an array

            self.temp = data[3]<<8|data[4]                        # Combine Temperature Bytes
            self.temp = (-45)+((175*self.temp)/65536)             # Convert to Celsius Value

            self.humidity = data[0]<<8|data[1]                    # Combine Humidity Bytes
            self.humidity = 100*self.humidity/65536               # Convert to % Humidity
            
# CCS811 Environmental Sensor
# Sparkfun Breakout Board            
class CCS811:
    def __init__(self):
        # Attempt to initialize device 
        try:
            self.addr = 0x5B                                    # Device Address
            self.meas_mode = 0x01                               # Measure Mode Register
            self.app_start = 0xF4                               # Application Start Register
            self.alg_result = 0x02                              # Algorithm Result Register
            
            self.eco2 = 0                                       # CO2 in parts per million (ppm)
            self.eTVOC = 0                                      # Total Volatile Organic Compounds in parts per billion (ppb)
            
            bus.write_byte(self.addr, self.app_start)           # Transition from boot to application start.
            bus.write_byte(self.addr, self.meas_mode)           # Select Measure Mode Register
            
            bus.write_i2c_block_data(self.addr,self.meas_mode, [0b00010000])          # Measurements once a second

            # Read data till no longer zero (should provide a minimum of 400ppm)
            while self.eco2 == 0:
                bus.write_byte(self.addr, self.alg_result)      # Change registers to algorithm result
                
                val = bus.read_i2c_block_data(self.addr,self.alg_result, 4)        # Read 4 bytes from algorithm result register
                
                self.eco2 = ((val[0] << 8) | (val[1]))          # Load first 2 bytes into CO2
                self.eTVOC = ((val[2] << 8) | (val[3]))         # Load last 2 bytes into TVOC
                time.sleep(.01)                                 # Sleep 10ms
                
            self._running = True                                # Initialization succeeded
                                                                # System will take data from sensor
            print("CCS811 Passed")                              # Let user know the sensor passed initialization.
            
        # Failure to initialize device.    
        except:
            self._running = False                               # Initialization failed
                                                                # System will not take data from sensor
            print("CCS811 Failed")                              # Let user know the sensor failed initialization.

    # Read Data
    def read_gas(self):
        if self._running:
            bus.write_byte(self.addr, self.alg_result)          # Change registers to algorithm result
            val = bus.read_i2c_block_data(0x5b, 0x02, 4)        # Read 4 bytes from algorithm results register
            self.eco2 = ((val[0] << 8) | (val[1]))              # Load first 2 bytes into CO2
            self.eTVOC = ((val[2] << 8) | (val[3]))             # Load last 2 bytes into TVOC


# BME280 Environmental Sensor
# Sparkfun Breakout Board
class bme280_sensor:
    def __init__(self):
        # Attempt to initialize device
        try:
            self.bme280_sensor = BME280(i2c_dev=bus, i2c_addr=0x77) # Initialize BME280 Library
            self.bme280_sensor.get_temperature()                    # Get Temperature to check if sensor is working
            self._running = True                                    # Initialization succeeded
                                                                    # System will take data from sensor
            print("BME280 Passed")                                  # Let user know the sensor passed initialization.
        except:
            self._running = False                                   # Initialization failed
                                                                    # System will not take data from sensor
            print("BME280 Failed")                                  # Let user know the sensor failed initialization.
    
    # Get Device Data
    def get_bme280_data(self):
        if self._running:
            self.temperature = self.bme280_sensor.get_temperature() # Get Temperature (C)
            self.pressure = 100*self.bme280_sensor.get_pressure()   # Get Pressure (Pa)
            self.humidity = self.bme280_sensor.get_humidity()       # Get Humidity (%)
            self.altitude = self.bme280_sensor.get_altitude()       # Get Altitude (m)

# MPR Series - MPRLS0025PA00001A Micro pressure Sensor
# Sparkfun Breakout Board
class microPressure:
    def __init__(self):
        # Attempt to initialize device
        try:
            self.addr = 0x18                                        # Device Address
            self.output_reg = 0xAA                                  # Output Command Byte
            self.data_reg = 0x00                                    # Command Byte
            self.Pressure = 0                                       # Pressure Data 
            
            msg=i2c_msg.write(self.addr,[self.output_reg,self.data_reg,self.data_reg]) # Write a message to receive data.
            bus.i2c_rdwr(msg)                                       # Send Message
            
            time.sleep(.1)                                          # Wait 10ms
            
            val=bus.read_byte(self.addr)                            # Write Address
            
            msg=i2c_msg.read(self.addr,4)                           # Write message to read 4 bytes from device
            bus.i2c_rdwr(msg)                                       # Send Message and receive 4 bytes
            
            self.data=list(msg)                                     # Convert message to array
            self.status=self.data[0]                                # Separate Status from data

            self._running = True                                    # Initialization succeeded
                                                                    # System will take data from sensor
            print("Micropressure Sensor Passed")                    # Let user know the sensor passed initialization.
        except:
            self._running = False                                   # Initialization failed
                                                                    # System will not take data from sensor
            print("Micropressure Sensor Failed")                    # Let user know the sensor failed initialization.

        
    # Read Pressure    
    def read_pressure(self):
        if self._running:
            msg=i2c_msg.write(self.addr,[self.output_reg,self.data_reg,self.data_reg]) # Write a message to receive data.
            bus.i2c_rdwr(msg)                                       # Send Message
            
            time.sleep(.1)                                          # Wait 10ms
            val=bus.read_byte(self.addr)                            # Write Address
            
            msg=i2c_msg.read(self.addr,4)                           # Write message to read 4 bytes from device
            bus.i2c_rdwr(msg)                                       # Send Message and receive 4 bytes
            content=list(msg)                                       # Convert message to array
            outputs=content[1]<<16 or content[2]<<8 or content[3]   # Combine Data Bytes

            # 10% to 90% calibration
            output_max=0xE66666
            output_min=0x19999A
            Pmax=25.000                                             # max psi
            Pmin=0.000                                              # min psi
            
            self.Pressure=(outputs-output_min)*(Pmax-Pmin)          # Pressure Calculation
            self.Pressure=(self.Pressure / (output_max-output_min))+Pmin # Pressure Calculation

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
        self.tmp=TMP117()
        self.shtc3=SHTC3()
        self.bme=bme280_sensor()
        self.ccs=CCS811()
        self.micro=microPressure()

       
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

