# Import Libraries / Modules
import os                                                       # Operating System Interfaces
from picamera import PiCamera                                   # Raspberry Pi Camera
from picamera import Color                                      # Raspberry Pi Camera
import threading                                                # Thread-Based Parallelism
import VL53L1X                                                  # Time-of-Flight sensor
import time                                                     # Time access and conversion module
from datetime import datetime                                   # Date and time module
from pymavlink import mavutil                                   # Communication for Ardupilot Mavlink Protocol
import numpy
import TMP117
import SHTC3
import CCS811
import self_bme280
import Micropressure

bus=4
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
            #print(self.tmp.temp_c)
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
            #print(self.ccs.eco2)
            #print(self.ccs.eTVOC)
            
    # What will run inside the thread        
    def run(self):
        while True:
            self.timer1=time.time()                                 # Mark current time
            
            # Read latest available data from all sensors
            self.micro.read_pressure()      # 10ms
            self.shtc3.measure() # 10-30 ms
            self.tmp.tempReading() # 1 ms
            self.bme.get_bme280_data() # 5-10 ms
            self.ccs.read_gas() # 1 ms
            
            # For timing purposes
            self.timer2=time.time()
            self.timeTotal=self.timer2-self.timer1
            print("Time to collect all sensors: ",self.timeTotal)
            
            self.fillData()                                        # Create data array from newly recorded sensors
           
            time.sleep(.05)                                         # Sleep 50ms
    
    # Attempt to initialize all possible sensor
    def checkConn(self):
        self.tmp=TMP117.TMP117(bus)
        self.shtc3=SHTC3.SHTC3(bus)
        self.bme=self_bme280.bme280_sensor() # Locked to Bus 4 for now.
        self.ccs=CCS811.CCS811(bus)
        self.micro=Micropressure.microPressure(bus)

       
class dataRecordBus (threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.name_file = datetime.now().strftime('%Y%m%d_%H%M%S')   # Create file name
        os.mkdir("/home/pi/Documents/logs/" + self.name_file)       # Create log directory
        self.log_file = self.name_file + "/" + self.name_file + ".txt" # Create log file
        self.video_file = "/home/pi/Documents/logs/" + self.name_file + "/" + self.name_file + ".h264" # Create video file
        
    def combineHeader(self,header1,header2):
        self.Header=sensor.header+navio.header
    # Thread main code
    def run(self):
        time.sleep(5)
        print(sensor.header)
        print(navio.header)
        self.combineHeader(sensor.header,navio.header)
        self.log_data_file(self.Header)                           # Log header data into log file
        while True:
            self.log_data_file(sensor.dataFull+navio.dataFull)                     # Log current sensor data
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
        

class Vehicle(threading.Thread):
    def __init__(self,frame):
        try:
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
                self.initializeFields(frame)
        except:
            self._running = False
            print("Mavlink Failed")
    def initializeFields(self,frame):
        #self.nav.mav.request_data_stream_send(1,0,mavutil.mavlink.MAV_DATA_STREAM_ALL,6,1)
        print("All Stream Halted")
        self.header=[]
        # Do Plane Initialization
        if frame == 'plane':
            # Set VFR HUD
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,2) # VFR_HUD, 2Hz
            print("VFR HUD Stream Started")
            self.VFR_HUD_HEADER=['mavpackettype', 'airspeed', 'groundspeed', 'heading', 'throttle', 'alt', 'climb']
            #self.VFR_HUD=['','','','','','','']
            self.header = self.header + self.VFR_HUD_HEADER
            
            #Set POSITION_TARGET_GLOBAL_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,2) #GPS_RAW_INT, 2 Hz
            print("POSITION_TARGET_GLOBAL_INT Stream Started")
            self.POSITION_TARGET_GLOBAL_INT_HEADER=['mavpackettype', 'time_boot_ms', 'coordinate_frame', 'type_mask',
                                                'lat_int', 'lon_int', 'alt', 'vx', 'vy', 'vz', 'afx', 'afy',
                                                'afz', 'yaw', 'yaw_rate']
            #self.POSITION_TARGET_GLOBAL_INT=['','','','','','','','','','','','','','','']
            self.header = self.header + self.POSITION_TARGET_GLOBAL_INT_HEADER
            
            # Set AHRS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS,2)
            print("AHRS Stream Started")
            self.AHRS_HEADER=['mavpackettype', 'omegaIx', 'omegaIy', 'omegaIz', 'accel_weight',
                          'renorm_val', 'error_rp', 'error_yaw']
            #self.AHRS=['','','','','','','','']
            self.header = self.header + self.AHRS_HEADER
            
            # Set GPS_RAW_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,2)
            print("GPS_RAW_INT Stream Started")
            self.GPS_RAW_INT_HEADER=['mavpackettype', 'time_usec', 'fix_type', 'lat', 'lon', 'alt',
                                 'eph', 'epv', 'vel', 'cog', 'satellites_visible']
            #self.GPS_RAW_INT=['','','','','','','','','','','']
            self.header = self.header + self.GPS_RAW_INT_HEADER
            
            # Set RC_CHANNELS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,2)
            print("RC_CHANNELS Stream Started")
            self.RC_CHANNELS_HEADER=['mavpackettype', 'time_boot_ms', 'chancount', 'chan1_raw',
                                 'chan2_raw', 'chan3_raw', 'chan4_raw', 'chan5_raw', 'chan6_raw',
                                 'chan7_raw', 'chan8_raw', 'chan9_raw', 'chan10_raw', 'chan11_raw',
                                 'chan12_raw', 'chan13_raw', 'chan14_raw', 'chan15_raw', 'chan16_raw',
                                 'chan17_raw', 'chan18_raw', 'rssi']
            #self.RC_CHANNELS=['','','','','','','','','','','','','','','','','','','','','','']
            self.header = self.header + self.RC_CHANNELS_HEADER
            
            # Set MISSION_CURRENT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,2)
            print("MISSION_CURRENT Stream Started")
            self.MISSION_CURRENT_HEADER=['mavpackettype', 'seq']
            #self.MISSION_CURRENT=['','']
            self.header = self.header + self.MISSION_CURRENT_HEADER
            
            # Set NAV_CONTROLLER_OUTPUT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,2)
            print("NAV_CONTROLLER_OUTPUT Stream Started")
            self.NAV_CONTROLLER_OUTPUT_HEADER=['mavpackettype', 'nav_roll', 'nav_pitch', 'nav_bearing',
                                           'target_bearing', 'wp_dist', 'alt_error', 'aspd_error', 'xtrack_error']
            #self.NAV_CONTROLLER_OUTPUT=['','','','','','','','','']
            self.header = self.header + self.NAV_CONTROLLER_OUTPUT_HEADER
            
            # Set SYS_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,2)
            print("SYS_STATUS Stream Started")
            self.SYS_STATUS_HEADER=['mavpackettype', 'onboard_control_sensors_present', 'onboard_control_sensors_enabled',
                                'onboard_control_sensors_health', 'load', 'voltage_battery', 'current_battery',
                                'battery_remaining', 'drop_rate_comm', 'errors_comm', 'errors_count1', 'errors_count2',
                                'errors_count3', 'errors_count4']
            #self.SYS_STATUS=['','','','','','','','','','','','','','']
            self.header = self.header + self.SYS_STATUS_HEADER
            
            # Set GLOBAL_POSITION_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,2)
            print("GLOBAL_POSITION_INT Stream Started")
            self.GLOBAL_POSITION_INT_HEADER=['mavpackettype', 'time_boot_ms', 'lat', 'lon', 'alt',
                                         'relative_alt', 'vx', 'vy', 'vz', 'hdg']
            #self.GLOBAL_POSITION_INT=['','','','','','','','','','']
            self.header = self.header + self.GLOBAL_POSITION_INT_HEADER
            
            # Set RAW_IMU
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,2)
            print("RAW_IMU Stream Started")
            self.RAW_IMU_HEADER=['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro',
                             'zgyro', 'xmag', 'ymag', 'zmag']
            #self.RAW_IMU=['','','','','','','','','','','']
            self.header = self.header + self.RAW_IMU_HEADER
            
            # Set BATTERY_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,2)
            self.BATTERY_STATUS_HEADER=['mavpackettype', 'id', 'battery_function', 'type', 'temperature',
                                    'voltages', 'current_battery', 'current_consumed', 'energy_consumed',
                                    'battery_remaining']
            #self.BATTERY_STATUS=['','','','','','','','','','']
            print("BATTERY_STATUS Stream Started")
            self.header = self.header + self.BATTERY_STATUS_HEADER
        elif frame=='rover':
            # Set VFR HUD
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,2) # VFR_HUD, 2Hz
            print("VFR HUD Stream Started")
            self.VFR_HUD_HEADER=['mavpackettype', 'airspeed', 'groundspeed', 'heading', 'throttle', 'alt', 'climb']
            self.VFR_HUD=self.createEmpty(self.VFR_HUD_HEADER)
            self.header = self.header + self.VFR_HUD_HEADER
            
            #Set POSITION_TARGET_GLOBAL_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,2) #GPS_RAW_INT, 2 Hz
            print("POSITION_TARGET_GLOBAL_INT Stream Started")
            self.POSITION_TARGET_GLOBAL_INT_HEADER=['mavpackettype', 'time_boot_ms', 'coordinate_frame', 'type_mask',
                                                'lat_int', 'lon_int', 'alt', 'vx', 'vy', 'vz', 'afx', 'afy',
                                                'afz', 'yaw', 'yaw_rate']
            self.POSITION_TARGET_GLOBAL_INT=self.createEmpty(self.POSITION_TARGET_GLOBAL_INT_HEADER)
            self.header = self.header + self.POSITION_TARGET_GLOBAL_INT_HEADER
            
            # Set AHRS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS,2)
            print("AHRS Stream Started")
            self.AHRS_HEADER=['mavpackettype', 'omegaIx', 'omegaIy', 'omegaIz', 'accel_weight',
                          'renorm_val', 'error_rp', 'error_yaw']
            self.AHRS=self.createEmpty(self.AHRS_HEADER)
            self.header = self.header + self.AHRS_HEADER
            
            # Set GPS_RAW_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,2)
            print("GPS_RAW_INT Stream Started")
            self.GPS_RAW_INT_HEADER=['mavpackettype', 'time_usec', 'fix_type', 'lat', 'lon', 'alt',
                                 'eph', 'epv', 'vel', 'cog', 'satellites_visible']
            self.GPS_RAW_INT=self.createEmpty(self.GPS_RAW_INT_HEADER)
            self.header = self.header + self.GPS_RAW_INT_HEADER
            
            # Set RC_CHANNELS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,2)
            print("RC_CHANNELS Stream Started")
            self.RC_CHANNELS_HEADER=['mavpackettype', 'time_boot_ms', 'chancount', 'chan1_raw',
                                 'chan2_raw', 'chan3_raw', 'chan4_raw', 'chan5_raw', 'chan6_raw',
                                 'chan7_raw', 'chan8_raw', 'chan9_raw', 'chan10_raw', 'chan11_raw',
                                 'chan12_raw', 'chan13_raw', 'chan14_raw', 'chan15_raw', 'chan16_raw',
                                 'chan17_raw', 'chan18_raw', 'rssi']
            self.RC_CHANNELS=self.createEmpty(self.RC_CHANNELS_HEADER)
            self.header = self.header + self.RC_CHANNELS_HEADER
            
            # Set MISSION_CURRENT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,2)
            print("MISSION_CURRENT Stream Started")
            self.MISSION_CURRENT_HEADER=['mavpackettype', 'seq']
            self.MISSION_CURRENT=self.createEmpty(self.MISSION_CURRENT_HEADER)
            self.header = self.header + self.MISSION_CURRENT_HEADER
            
            # Set NAV_CONTROLLER_OUTPUT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,2)
            print("NAV_CONTROLLER_OUTPUT Stream Started")
            self.NAV_CONTROLLER_OUTPUT_HEADER=['mavpackettype', 'nav_roll', 'nav_pitch', 'nav_bearing',
                                           'target_bearing', 'wp_dist', 'alt_error', 'aspd_error', 'xtrack_error']
            self.NAV_CONTROLLER_OUTPUT=self.createEmpty(self.NAV_CONTROLLER_OUTPUT_HEADER)
            self.header = self.header + self.NAV_CONTROLLER_OUTPUT_HEADER
            
            # Set SYS_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,2)
            print("SYS_STATUS Stream Started")
            self.SYS_STATUS_HEADER=['mavpackettype', 'onboard_control_sensors_present', 'onboard_control_sensors_enabled',
                                'onboard_control_sensors_health', 'load', 'voltage_battery', 'current_battery',
                                'battery_remaining', 'drop_rate_comm', 'errors_comm', 'errors_count1', 'errors_count2',
                                'errors_count3', 'errors_count4']
            self.SYS_STATUS=self.createEmpty(self.SYS_STATUS_HEADER)
            self.header = self.header + self.SYS_STATUS_HEADER
            
            # Set GLOBAL_POSITION_INT
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,2)
            print("GLOBAL_POSITION_INT Stream Started")
            self.GLOBAL_POSITION_INT_HEADER=['mavpackettype', 'time_boot_ms', 'lat', 'lon', 'alt',
                                         'relative_alt', 'vx', 'vy', 'vz', 'hdg']
            self.GLOBAL_POSITION_INT=self.createEmpty(self.GLOBAL_POSITION_INT_HEADER)
            self.header = self.header + self.GLOBAL_POSITION_INT_HEADER
            
            # Set RAW_IMU
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,2)
            print("RAW_IMU Stream Started")
            self.RAW_IMU_HEADER=['mavpackettype', 'time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro',
                             'zgyro', 'xmag', 'ymag', 'zmag']
            self.RAW_IMU=self.createEmpty(self.RAW_IMU_HEADER)
            self.header = self.header + self.RAW_IMU_HEADER
            
            # Set BATTERY_STATUS
            self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,2)
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
            
            #self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_CURRENT_WAYPOINT,2)
            
            
    def createEmpty(self, keys):
        return {keys[x]:'' for x in range(len(keys))}
    
    def set_Message_Interval(self,message_id, frequency_Hz):
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
        if self._running:
            self.msg=self.nav.recv_match().to_dict()
            #print(self.msg)
    def createData(self):
        self.dataFull = list(self.VFR_HUD.values())+list(self.POSITION_TARGET_GLOBAL_INT.values())+list(self.AHRS.values())+list(self.GPS_RAW_INT.values()) + list(self.RC_CHANNELS.values()) + list(self.MISSION_CURRENT.values()) + list(self.NAV_CONTROLLER_OUTPUT.values()) + list(self.SYS_STATUS.values()) + list(self.GLOBAL_POSITION_INT.values()) + list(self.RAW_IMU.values())+list(self.BATTERY_STATUS.values())+list(self.HEARTBEAT.values())
        #print(self.dataFull)
    def parseMessage(self):
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
            else:
                pass
    
    def run(self):
        if self._running:
            while self._running:
                time.sleep(.05)
                #print(navio.nav.messages)
                self.getMessage()
                self.parseMessage()
                self.createData()
                

sensor=sensorBus()
navio=Vehicle('rover')
record=dataRecordBus()
sensor.start()
navio.start()
record.start()

