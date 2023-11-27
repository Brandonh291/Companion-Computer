# Import Libraries / Modules
import os                                                       # Operating System Interfaces
from picamera import PiCamera                                   # Raspberry Pi Camera
from picamera import Color                                      # Raspberry Pi Camera
import threading                                                # Thread-Based Parallelism
import VL53L1X                                                  # Time-of-Flight sensor
import time                                                     # Time access and conversion module
from datetime import datetime                                   # Date and time module
from pymavlink import mavutil                                   # Communication for Ardupilot Mavlink Protocol
import random
import numpy
import TMP117
import SHTC3
import CCS811
import self_bme280
import Micropressure

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
        
class Vehicle(threading.Thread):
    """
    Class representing a vehicle with MAVLink communication capabilities.

    Parameters:
    - frame (str): The type of vehicle frame ('plane', 'rover', etc.).

    Attributes:
    - exit (bool): Flag indicating whether the vehicle thread should exit.
    - _armed (bool): Flag indicating whether the vehicle is armed.
    - header (list): List to store headers of different MAVLink message types.
    - dataFull (list): List to store data from MAVLink messages.
    - frame (str): Type of vehicle frame.
    - heart (bool): Flag indicating whether a heartbeat signal has been received.
    - nav (mavutil.mavlink_connection): MAVLink connection object.
    - heartVal (mavutil.mavlink_heartbeat): Heartbeat message received from the system.
    - _running (bool): Flag indicating whether the vehicle thread is running.

    Methods:
    - __init__(self, frame): Constructor method for the Vehicle class.
    - attemptConnection(self): Attempts to establish a MAVLink connection with the vehicle.
    - initializeFields(self): Initializes MAVLink message types and sets message intervals based on the vehicle frame.
    - createEmpty(self, keys): Creates an empty dictionary with keys initialized to empty strings.
    - set_Message_Interval(self, message_id, frequency_Hz): Sets the message interval for a specific MAVLink message type.
    - getMessage(self): Retrieves and stores a MAVLink message from the vehicle.
    - createData(self): Creates a list of data from various MAVLink messages if a heartbeat is received and the vehicle is armed.
    - parseMessage(self): Parses the received MAVLink message and updates relevant attributes based on the message type.
    - run(self): Main execution method for the vehicle thread, continuously collects and processes MAVLink messages.
    """
    def __init__(self,frame):
        try:
            self.exit=False
            self._armed=False
            self.header=[]
            self.dataFull=[]
            self.frame=frame
            self.heart=False 
            threading.Thread.__init__(self)
            self.nav = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
            print("Waiting for Heartbeat") # We need to receive a correct signal from the controller to indicate we are atleast receiving data
        
            self.heartVal = self.nav.wait_heartbeat(timeout=30) # Waiting 30 seconds for a heart message
            if self.heartVal == None:
                print("Failure")
                self._running = False # If we receive no message, then set vehicle to not running so we dont mess up asking for additonal commands
            
            else:
                self._running = True
                print("Heartbeat from system: ",self.nav.
                      target_system," and component: ", self.nav.target_component)
                print(self.heartVal)
                self.initializeFields()
        except:
            self._running = False

    def attemptConnection(self):
        """
        Attempts to establish a MAVLink connection with the vehicle.

        If successful, initializes necessary attributes and starts MAVLink message streams.
        """
        try:
            self.nav = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
            print("Waiting for Heartbeat") # We need to receive a correct signal from the controller to indicate we are atleast receiving data
        
            self.heartVal = self.nav.wait_heartbeat(timeout=30) # Waiting 30 seconds for a heart message
            if self.heartVal == None:
                print("Failure to find Heartbeat")
                self._running = False # If we receive no message, then set vehicle to not running so we dont mess up asking for additonal commands
            
            else:
                self._running = True
                print("Heartbeat from system: ",self.nav.
                      target_system," and component: ", self.nav.target_component)
                print(self.heartVal)
                self.initializeFields()
        except:
            self._running = False
            print("Mavlink Error in Attempting Connection")

    def initializeFields(self):
        """
        Initializes MAVLink message types and sets message intervals based on the vehicle frame.
        """
        #self.nav.mav.request_data_stream_send(1,0,mavutil.mavlink.MAV_DATA_STREAM_ALL,6,1)
        print("All Stream Halted")
        # Do Plane Initialization
        if self.frame == 'plane' or self.frame == 'rover':
            # Set VFR HUD
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,1) # VFR_HUD, 2Hz
            print("VFR HUD Stream Started")
            self.VFR_HUD_HEADER=['mavpackettype', 'airspeed', 'groundspeed', 'heading', 'throttle', 'alt', 'climb']
            self.VFR_HUD=self.createEmpty(self.VFR_HUD_HEADER)
            self.header = self.header + self.VFR_HUD_HEADER
            
            if self.frame == 'plane':
                #Set POSITION_TARGET_GLOBAL_INT
                self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,1) #GPS_RAW_INT, 2 Hz
                print("POSITION_TARGET_GLOBAL_INT Stream Started")
                self.POSITION_TARGET_GLOBAL_INT_HEADER=['mavpackettype', 'time_boot_ms', 'coordinate_frame', 'type_mask',
                                                    'lat_int', 'lon_int', 'alt', 'vx', 'vy', 'vz', 'afx', 'afy',
                                                    'afz', 'yaw', 'yaw_rate']
                self.POSITION_TARGET_GLOBAL_INT=self.createEmpty(self.POSITION_TARGET_GLOBAL_INT_HEADER)
                self.header = self.header + self.POSITION_TARGET_GLOBAL_INT_HEADER
            
            # Set AHRS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS,1)
            print("AHRS Stream Started")
            self.AHRS_HEADER=['mavpackettype', 'omegaIx', 'omegaIy', 'omegaIz', 'accel_weight',
                          'renorm_val', 'error_rp', 'error_yaw']
            self.AHRS=self.createEmpty(self.AHRS_HEADER)
            self.header = self.header + self.AHRS_HEADER
            
            # Set GPS_RAW_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,1)
            print("GPS_RAW_INT Stream Started")
            self.GPS_RAW_INT_HEADER=['mavpackettype', 'time_usec', 'fix_type', 'lat', 'lon', 'alt',
                                 'eph', 'epv', 'vel', 'cog', 'satellites_visible']
            # This has done two different types of sets now and im not sure which is which.
            self.GPS_RAW_INT=self.createEmpty(self.GPS_RAW_INT_HEADER)
            self.header = self.header + self.GPS_RAW_INT_HEADER
            
            # Set RC_CHANNELS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,1)
            print("RC_CHANNELS Stream Started")
            self.RC_CHANNELS_HEADER=['mavpackettype', 'time_boot_ms', 'chancount', 'chan1_raw',
                                 'chan2_raw', 'chan3_raw', 'chan4_raw', 'chan5_raw', 'chan6_raw',
                                 'chan7_raw', 'chan8_raw', 'chan9_raw', 'chan10_raw', 'chan11_raw',
                                 'chan12_raw', 'chan13_raw', 'chan14_raw', 'chan15_raw', 'chan16_raw',
                                 'chan17_raw', 'chan18_raw', 'rssi']
            self.RC_CHANNELS=self.createEmpty(self.RC_CHANNELS_HEADER)
            self.header = self.header + self.RC_CHANNELS_HEADER
            
            # Set MISSION_CURRENT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,1)
            print("MISSION_CURRENT Stream Started")
            self.MISSION_CURRENT_HEADER=['mavpackettype', 'seq']
            self.MISSION_CURRENT=self.createEmpty(self.MISSION_CURRENT_HEADER)
            self.header = self.header + self.MISSION_CURRENT_HEADER
            
            if self.frame == 'plane':
                # Set NAV_CONTROLLER_OUTPUT
                self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,1)
                print("NAV_CONTROLLER_OUTPUT Stream Started")
                self.NAV_CONTROLLER_OUTPUT_HEADER=['mavpackettype', 'nav_roll', 'nav_pitch', 'nav_bearing',
                                               'target_bearing', 'wp_dist', 'alt_error', 'aspd_error', 'xtrack_error']
                self.NAV_CONTROLLER_OUTPUT=self.createEmpty(self.NAV_CONTROLLER_OUTPUT_HEADER)
                self.header = self.header + self.NAV_CONTROLLER_OUTPUT_HEADER
            
            # Set SYS_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,1)
            print("SYS_STATUS Stream Started")
            self.SYS_STATUS_HEADER=['mavpackettype', 'onboard_control_sensors_present', 'onboard_control_sensors_enabled',
                                'onboard_control_sensors_health', 'load', 'voltage_battery', 'current_battery',
                                'battery_remaining', 'drop_rate_comm', 'errors_comm', 'errors_count1', 'errors_count2',
                                'errors_count3', 'errors_count4']
            self.SYS_STATUS=self.createEmpty(self.SYS_STATUS_HEADER)
            self.header = self.header + self.SYS_STATUS_HEADER
            
            # Set GLOBAL_POSITION_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,1)
            print("GLOBAL_POSITION_INT Stream Started")
            self.GLOBAL_POSITION_INT_HEADER=['mavpackettype', 'time_boot_ms', 'lat', 'lon', 'alt',
                                         'relative_alt', 'vx', 'vy', 'vz', 'hdg']
            self.GLOBAL_POSITION_INT=self.createEmpty(self.GLOBAL_POSITION_INT_HEADER)
            self.header = self.header + self.GLOBAL_POSITION_INT_HEADER
            
            # Set RAW_IMU
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,1)
            print("RAW_IMU Stream Started")
            self.RAW_IMU_HEADER=['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro',
                             'zgyro', 'xmag', 'ymag', 'zmag']
            self.RAW_IMU=self.createEmpty(self.RAW_IMU_HEADER)
            self.header = self.header + self.RAW_IMU_HEADER
            
            # Set BATTERY_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,1)
            self.BATTERY_STATUS_HEADER=['mavpackettype', 'id', 'battery_function', 'type', 'temperature',
                                    'voltages', 'current_battery', 'current_consumed', 'energy_consumed',
                                    'battery_remaining']
            self.BATTERY_STATUS=self.createEmpty(self.BATTERY_STATUS_HEADER)
            print("BATTERY_STATUS Stream Started")
            self.header = self.header + self.BATTERY_STATUS_HEADER
            
            self.HEARTBEAT_HEADER=['mavpackettype','type','autopilot','base_mode','custom_mode','system_status','mavlink_version']
            self.HEARTBEAT=self.createEmpty(self.HEARTBEAT_HEADER)
            print("Heartbeat Stream Started")
            self.header = self.header + self.HEARTBEAT_HEADER

    def createEmpty(self, keys):
        """
        Creates an empty dictionary with keys initialized to empty strings.

        Parameters:
        - keys (list): List of keys for the dictionary.

        Returns:
        - dict: Empty dictionary with keys.
        """
        return {keys[x]:'' for x in range(len(keys))}

    def set_Message_Interval(self, message_id, frequency_Hz):
        """
        Sets the message interval for a specific MAVLink message type.

        Parameters:
        - message_id (int): ID of the MAVLink message.
        - frequency_Hz (float): Desired frequency of the message in Hertz.
        """
        if self._running:
            self.nav.mav.command_long_send(1,self.nav.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                       0,
                                       message_id,
                                       1e6/frequency_Hz,
                                       0,0,0,0,
                                       2)
            time.sleep(0.1)

    def getMessage(self):
        """
        Retrieves and stores a MAVLink message from the vehicle.
        """
        if self._running:
            self.msg=self.nav.recv_match().to_dict()
            #print(self.msg)

    def createData(self):
        """
        Creates a list of data from various MAVLink messages if a heartbeat is received and the vehicle is armed.
        """
        if self.heart:
            if self._armed:
                self.dataFull = list(self.VFR_HUD.values()) 
                if self.frame=='plane':
                    self.dataFull= self.dataFull + list(self.POSITION_TARGET_GLOBAL_INT.values())
                self.dataFull=self.dataFull +list(self.AHRS.values())+list(self.GPS_RAW_INT.values()) + list(self.RC_CHANNELS.values()) + list(self.MISSION_CURRENT.values())
                if self.frame == 'plane':
                    self.dataFull = self.dataFull + list(self.NAV_CONTROLLER_OUTPUT.values())
                self.dataFull = self.dataFull + list(self.SYS_STATUS.values()) + list(self.GLOBAL_POSITION_INT.values()) + list(self.RAW_IMU.values())+list(self.BATTERY_STATUS.values())+list(self.HEARTBEAT.values())
        #print(self.dataFull)

    def parseMessage(self):
        """
        Parses the received MAVLink message and updates relevant attributes based on the message type.
        """
        if self._running:
            if self.msg['mavpackettype']=='VFR_HUD':
                #print(self.msg)
                self.VFR_HUD=self.msg
            elif self.msg['mavpackettype']=='POSITION_TARGET_GLOBAL_INT':
                #print(self.msg)
                self.POSITION_TARGET_GLOBAL_INT=self.msg
            elif self.msg['mavpackettype']=='AHRS':
                #print(self.msg)
                self.AHRS=self.msg
            elif self.msg['mavpackettype']=='GPS_RAW_INT':
                #print(self.msg)
                self.GPS_RAW_INT=self.msg
            elif self.msg['mavpackettype']=='RC_CHANNELS':
                #print(self.msg)
                self.RC_CHANNELS=self.msg
            elif self.msg['mavpackettype']=='MISSION_CURRENT':
                #print(self.msg)
                self.MISSION_CURRENT=self.msg
            elif self.msg['mavpackettype']=='NAV_CONTROLLER_OUTPUT':
                #print(self.msg)
                self.NAV_CONTROLLER_OUTPUT=self.msg
            elif self.msg['mavpackettype']=='SYS_STATUS':
                #print(self.msg)
                self.SYS_STATUS=self.msg
            elif self.msg['mavpackettype']=='GLOBAL_POSITION_INT':
                #print(self.msg)
                self.GLOBAL_POSITION_INT=self.msg
            elif self.msg['mavpackettype']=='RAW_IMU':
                #print(self.msg)
                self.RAW_IMU=self.msg
            elif self.msg['mavpackettype']=='BATTERY_STATUS':
                #print(self.msg)
                self.BATTERY_STATUS=self.msg
            elif self.msg['mavpackettype']=='HEARTBEAT':
                #print(self.msg)
                self.HEARTBEAT=self.msg
                self.heart=True
                if self.HEARTBEAT['type'] == 10 and self.heart== True and self.HEARTBEAT['base_mode']>128:
                    self._armed=True
                    #print("Armed")
                elif self.HEARTBEAT['type'] == 10 and self.heart== True and self.HEARTBEAT['base_mode']<128:
                    self._armed=False
                    #print("Disarmed")
                else:
                    pass
                #print(self._armed)
            else:
                pass

    def run(self):
        """
        Main execution method for the vehicle thread.

        Continuously collects and processes MAVLink messages while the thread is running.
        """
        while not self.exit:
            if self._running:
                nextTime=time.time()
                while self._running:
                    curTime=time.time()
                    if curTime-nextTime > 5:
                        if self._armed:
                            self.nav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"System Collecting Data".encode())
                            #print("Hellooo")
                        else:
                            self.nav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"System Not Collecting Data".encode())

                        nextTime=time.time()
                    #time.sleep(.01)
                    #print(navio.nav.messages)
                    try:
                        self.getMessage()
                    except:
                        pass
                    self.parseMessage()
                    self.createData()
            if not self._running:
                time.sleep(60)
                self.attemptConnection()

                
                

sensor=SensorBus()
navio=Vehicle('rover')
record=DataRecordBus()
sensor.start()
navio.start()
record.start()
restart=0


