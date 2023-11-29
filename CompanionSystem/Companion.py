# Import Libraries / Modules
import os                                                       # Operating System Interfaces
from picamera import PiCamera                                   # Raspberry Pi Camera
from picamera import Color                                      # Raspberry Pi Camera
import threading                                                # Thread-Based Parallelism
import VL53L1X                                                  # Time-of-Flight sensor
import time                                                     # Time access and conversion module
from datetime import datetime                                   # Date and time module
import random
import TMP117
import SHTC3
import CCS811
import self_bme280
import Micropressure
import MavVehicle as Vehicle
bus=3
startTime=time.time()                                           # Use to mark a starting time for code, used for timing and data recording.

# from sensors import TMP117, SHTC3, BME280, CCS811, Micropressure This might be an interesting thing to do where we put all sensors in a folder, and either a folder within that?

class SensorBus(threading.Thread):
    """
    SensorBus class for managing multiple sensors in a threaded environment.

    Attributes:
    - checkArmed (int): Flag to check if the system is armed.
    - dataFull (list): List to store recorded data.
    - header (list): List to store header information.
    - timer1 (float): Timer for marking the current time.
    - timer2 (float): Timer for marking the end time.
    - timeTotal (float): Total time taken for sensor readings.
    - _running (bool): Indicates whether the thread is running.

    Methods:
    - __init__(self): Initialize the SensorBus.
    - makeHeader(self): Create a header for recorded data based on initialized sensors.
    - fillData(self): Create a data array to be recorded based on initialized sensors.
    - run(self): Threaded function for continuously reading sensor data.
    - checkConn(self): Attempt to initialize all possible sensors.
    """

    def __init__(self):
        """
        Initialize the SensorBus.
        """
        threading.Thread.__init__(self)
        self.checkArmed = 0
        self.dataFull = []  # Initialize data array
        self.header = []  # initialize header array
        self.makeHeader()  # Create header
        self._running = True

    def makeHeader(self):
        """
        Create header for recorded data based on what sensors passed initialization.
        """
        self.checkConn()  # Attempt to initialize all possible sensors
        self.header = []
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

    def fillData(self):
        """
        Create Data array to be recorded based on what sensors passed initialization.
        """
        self.dataFull = []
        self.dataFull.append((self.timer1 - startTime))
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

    def run(self):
        """
        Threaded function for continuously reading sensor data.
        """
        while self._running:
            self.timer1 = time.time()  # Mark current time
            if not navio._armed:
                self.checkArmed = 1

            if navio._armed and self.checkArmed == 1:
                self.makeHeader()
                self.checkArmed = 0
            if navio._armed or not navio._running:
                # Read latest available data from all sensors
                self.micro.read_pressure()  # 10ms
                self.shtc3.measure()  # 10-30 ms
                self.tmp.tempReading()  # 1 ms
                self.bme.get_bme280_data()  # 5-10 ms
                self.ccs.read_gas()  # 1 ms

                # For timing purposes
                self.timer2 = time.time()
                self.timeTotal = self.timer2 - self.timer1

                self.fillData()  # Create data array from newly recorded sensors

            time.sleep(0.05)  # Sleep 50ms

    def checkConn(self):
        """
        Attempt to initialize all possible sensors.
        """
        self.tmp = TMP117.TMP117(bus)
        self.shtc3 = SHTC3.SHTC3(bus)
        self.bme = self_bme280.BME280_Sensor()
        self.ccs = CCS811.CCS811(bus)
        self.micro = Micropressure.microPressure(bus)

       
class DataRecordBus(threading.Thread):
    """
    DataRecordBus class for managing data recording in a threaded environment.

    Attributes:
    - name_file (str): Name of the log file and directory.
    - log_file (str): Path to the log file.
    - video_file (str): Path to the video file.
    - makeNew (bool): Flag to indicate whether a new log should be created.
    - newCount (int): Counter for creating a new log.
    - _running (bool): Indicates whether the thread is running.

    Methods:
    - __init__(self): Initialize the DataRecordBus.
    - makeFile(self): Create a new log file and directory.
    - combineHeader(self, header1, header2): Combine headers from different sources.
    - run(self): Threaded function for continuously recording sensor data.
    - log_data_file(self, data): Save data in the log text file.
    """

    def __init__(self):
        """
        Initialize the DataRecordBus.
        """
        threading.Thread.__init__(self)
        self.makeFile()
        self.newCount = 0
        self._running = True

    def makeFile(self):
        """
        Create a new log file and directory.
        """
        global startTime
        startTime = time.time()
        self.name_file = datetime.now().strftime('%Y%m%d_%H%M%S')  # Create file name
        random.seed()
        self.name_file = self.name_file + "_" + str(random.randrange(0, 10001, 2))
        os.mkdir("/home/pi/Documents/logs/" + self.name_file)  # Create log directory
        self.log_file = self.name_file + "/" + self.name_file + ".txt"  # Create log file
        self.video_file = "/home/pi/Documents/logs/" + self.name_file + "/" + self.name_file + ".h264"  # Create video file
        self.makeNew = False

    def combineHeader(self, header1, header2):
        """
        Combine headers from different sources.

        Args:
        - header1 (list): Header from the first source.
        - header2 (list): Header from the second source.
        """
        self.Header = sensor.header + navio.header

    def run(self):
        """
        Threaded function for continuously recording sensor data.
        """
        time.sleep(10)
        print(sensor.header)
        print(navio.header)
        self.combineHeader(sensor.header, navio.header)
        self.log_data_file(self.Header)  # Log header data into log file
        while self._running:
            if navio._running:
                if not navio._armed and self.makeNew == False:
                    self.newCount = self.newCount + 1
                    if self.newCount > 5:
                        self.makeNew = True
                        self.newCount = 0
                        print("NEW LOG")
                if self.makeNew and navio._armed:
                    self.makeFile()
                    print(sensor.header)
                    print(navio.header)
                    self.combineHeader(sensor.header, navio.header)
                    self.log_data_file(self.Header)  # Log header data into log file
                    print("New Log Made")
            if ((navio._running == False) or navio._armed):
                print(sensor.dataFull)
                self.log_data_file(sensor.dataFull + navio.dataFull)  # Log current sensor data
            else:
                print("System Disarmed and Navio Connected, No Logging")
                time.sleep(0.5)
            time.sleep(0.2)  # Sleep 200ms

    def log_data_file(self, data):
        """
        Save data in the log text file.

        Args:
        - data (list): Data to be logged.
        """
        file = open(r"/home/pi/Documents/logs/" + self.log_file, "a")  # open log file
        for L in range(len(data)):  # Append data
            if L == len(data) - 1:
                file.write(str(data[L]) + "\r\n")
            else:
                file.write(str(data[L]) + ";")
        file.close()  # Close file

                
                

sensor=SensorBus()
navio=Vehicle.Vehicle('rover')
record=DataRecordBus()
sensor.start()
navio.start()
record.start()
restart=0