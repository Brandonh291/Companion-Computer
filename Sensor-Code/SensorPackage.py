#######################################################
# Current Date of Version: 1/27/2022
#######################################################
# Libraries
import os
from picamera import PiCamera
import threading
import VL53L1X
import time
from smbus2 import SMBus
from datetime import datetime
from bme280_sensor import BME280
from pymavlink import mavutil

##############################################
logged_data = ["time(s"]  # Array for Data that will be logged
data = []
########################################################
# Threading Testing
bus = SMBus(4)


class CCS811:
    def __init__(self):
        #  # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        # try activating the CCS811 Sensor
        try:
            global eco2
            global eTVOC
            # Will need to wait 20 minutes for decent readings

            # Done at Startup, exit boot
            bus.write_byte(0x5B, 0xF4)

            # Set Mode
            bus.write_byte(0x5B, 0x01)
            bus.write_i2c_block_data(0x5B, 0x01, [0b00010000])

            # Go to Data
            # Ignore first 3, they are calibrations
            # bus.write_byte(0x5B,0x02)
            eco2 = 0
            while eco2 == 0:
                bus.write_byte(0x5B, 0x02)
                val = bus.read_i2c_block_data(0x5b, 0x02, 4)
                eco2 = ((val[0] << 8) | (val[1]))
                eTVOC = ((val[2] << 8) | (val[3]))
                time.sleep(1)
            logged_data.append("CCS811 CO2 (ppm)")
            logged_data.append("CCS811 tVOC (ppb)")
            self._running = True

        except:
            print("CCS811 not Detected")
            self._running = False
            return

    def read_gas(self):
        global eco2
        global eTVOC
        if self._running:
            global data
            # Go to Data
            # Ignore first 3, they are calibrations
            bus.write_byte(0x5B, 0x02)
            val = bus.read_i2c_block_data(0x5b, 0x02, 4)
            eco2 = ((val[0] << 8) | (val[1]))
            eTVOC = ((val[2] << 8) | (val[3]))
            data.append(eco2)
            data.append(eTVOC)


class bme280_sensor:
    def __init__(self):
        #  # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        # Acivate BME280 I2C
        try:
            global bme280_sensor
            bme280_sensor = BME280(i2c_dev=bus, i2c_addr=0x77)
            logged_data.append("BME280 Temperature (C)")
            logged_data.append("BME280 Pressure (hPa)")
            logged_data.append("BME280 Humidity (%)")
            logged_data.append("BME280 Altitude (ft)")
            self._running = True
        except:
            print("BME280 not Detected")
            self._running = False
            return

    def get_bme280_data(self):
        try:
            if self._running:
                global data
                temperature = bme280_sensor.get_temperature()
                data.append(temperature)
                pressure = bme280_sensor.get_pressure()
                data.append(pressure)
                humidity = bme280_sensor.get_humidity()
                data.append(humidity)
                altitude = bme280_sensor.get_altitude()
                data.append(altitude)
        except:
            if self._running:
                print("Connection to Environmental Sensor Lost")
                append_blanks(6)


def append_blanks(num_blank):
    global data
    for i in range(num_blank):
        data.append("")


class VL53l1x_distance_sensor:
    def __init__(self):
        # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        try:
            global tof
            bus.read_byte_data(0x29, 0x00)
            print("Received a Response")
            tof = VL53L1X.VL53L1X(i2c_bus=4, i2c_address=0x29)
            tof.open()
            logged_data.append("VL53L1X Distance (mm)")
            self._running = True
        except:
            print("VL53L1X Sensor not Detected")
            self._running = False
            return

    def measure_distance(self):
        try:
            if self._running:
                global data
                tof.start_ranging(1)
                distance = tof.get_distance()
                # print(distance)
                tof.stop_ranging()
                data.append(distance)
                return distance
        except:
            if self._running:
                print("Connection to VL53L1X Lost")
                append_blanks(1)


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
    def twos_comp(self, val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def tempReading(self):
        try:
            if self._running:
                # Initialize I2C (SMBus)

                global data
                # TMP117 Registers
                tmp117_addr = 0x48
                tmp117_reg_temp = 0x00
                tmp117_reg_config = 0x01
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
                append_blanks(1)


class FullLog:
    def __init__(self):
        # Performance initalization
        # Put the created log document in here
        # probably initalized each arm, or create a re_initialize function
        self.name_file = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.mkdir("/home/pi/Documents/logs/" + name_file)
        self.video_file = "/home/pi/Documents/logs/" + name_file + "/" + name_file + ".h264"
        self.log_file = name_file + "/" + name_file + ".txt"

    def log_data_file(self, data):
        # Log Current Data
        # log data to an file as an append
        file = open(r"/home/pi/Documents/logs/" + self.log_file, "a")
        for L in range(len(data)):
            if L == len(data) - 1:
                file.write(str(data[L]) + "\r\n")
            else:
                file.write(str(data[L]) + ",")
        file.close()

    def initialize_log_file(self):
        # might be the new function for re-initializing
        # log data to an file as an append
        self.name_file = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.mkdir("/home/pi/Documents/logs/" + self.name_file)
        self.video_file = "/home/pi/Documents/logs/" + self.name_file + "/" + self.name_file + ".h264"
        self.log_file = self.name_file + "/" + self.name_file + ".txt"


class MavConnection:
    def __init__(self, time_gap=1):
        global master

        try:
            master = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
            print("Waiting for Heartbeat")
            val = master.wait_heartbeat(timeout=10)  # Try adding a timeout to see if we actually get through or not
            print(val)
            # time.sleep(5)
            if val == None:
                self._running = False
            else:
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
                self.nav_time = ''
                self.nav_lat = ''
                self.nav_long = ''
                self.nav_rel_alt = ''
                self.nav_alt = ''
                self.nav_vx = ''
                self.nav_vy = ''
                self.nav_vz = ''
                self.nav_hdg = ''
                self.bad_data = 0
                self.check_once = 0
                self.base_time = 0
                self.overtime = 0
                self.over_time = 0
                self.time_gap = time_gap
                self.heartBeatCount = 0
                self.heartBeatArmed = 129
                self._running = True
                self.initialize_mavlink_log()
        except:
            print("Failed to Initialize Mavlink")
            self.running = False

    def initialize_mavlink_log(self):
        if self._running:
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

    def clear_buffer(self):
        if self._running:
            overtime = 1
            while overtime == 1:
                check = master.recv_match().to_dict()
                try:
                    self.over_time = check['time_boot_ms']
                except:
                    pass
                if (data[0] - (self.over_time - self.base_time) / 1000) < self.time_gap:
                    overtime = 0

    def get_message(self):
        global check_once
        global bad_data
        global data
        global heartBeatArmed

        if self._running:
            try:
                msg = master.recv_match().to_dict()
                print(msg)
                time.sleep(.25)

                try:
                    self.over_time = msg['time_boot_ms']
                except:
                    print("Overtime Exception")
                    pass

                if test_mode == 1:
                    print(msg['mavpackettype'])

                try:
                    if msg['mavpackettype'] == 'BAD_DATA':
                        self.bad_data = self.bad_data + 1
                        print("Consecutive Bad Data: ", bad_data)
                        print(msg)
                except:
                    print("Bad Data Exception")
                    pass

                try:
                    if self.check_once == 0:
                        self.base_time = msg['time_boot_ms']
                        self.check_once = 1
                except:
                    print("Check Once Exception")
                    pass

                try:
                    if (data[0] - (self.over_time - self.base_time) / 1000) > 5:
                        self.clear_buffer()
                except:
                    print("Clear Buffer Exception")
                    pass

                if msg['mavpackettype'] == 'NAV_CONTROLLER_OUTPUT':
                    try:
                        print(msg['wp_dist'])
                    except:
                        print("Time Failed to Load")

                if msg['mavpackettype'] == 'HEARTBEAT':
                    try:
                        # print(msg)
                        if msg['base_mode'] < 190:
                            self.heartBeatCount = self.heartBeatCount + 1
                            print("Count Incremented")
                        else:
                            self.heartBeatCount = 0
                            print("Count Reset")

                        if self.heartBeatCount > 10:
                            self.heartBeatArmed = 0
                        if msg['base_mode'] > 100:
                            self.heartBeatArmed = 129
                        # heartBeatArmed=msg['base_mode']
                        print(self.heartBeatCount)
                    except:
                        print("Heart Beat Exception")
                        pass

                if msg['mavpackettype'] == 'GLOBAL_POSITION_INT':
                    try:
                        bad_data = 0
                        self.nav_time = msg['time_boot_ms']
                        self.nav_lat = msg['lat']
                        self.nav_long = msg['lon']
                        self.nav_rel_alt = msg['relative_alt']
                        self.nav_alt = msg['alt']
                        self.nav_vx = msg['vx']
                        self.nav_vy = msg['vy']
                        self.nav_vz = msg['vz']
                        self.nav_hdg = msg['hdg']
                        # print(msg['time_boot_ms'])
                    except:
                        print("Global Position Failed ")
            except:
                print("Exception in Mavlink")
                # append_Blanks(9)

    def append_data(self):
        if self._running:
            data.append(self.nav_time)
            data.append(self.nav_lat)
            data.append(self.nav_long)
            data.append(self.nav_rel_alt)
            data.append(self.nav_alt)
            data.append(self.nav_vx)
            data.append(self.nav_vy)
            data.append(self.nav_vz)
            data.append(self.nav_hdg)


def camera_record():
    global Mavlink
    global name_file
    global video_file
    global log_file
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 20
    camera.start_recording(video_file)
    print("Camera Recording")
    while Mavlink.heartBeatArmed == 129 or test_mode == 1:
        print("recording stuff")
        time.sleep(1)
    camera.stop_recording()
    camera.close()


#########################################
# Adjustable Variables
test_mode = 0  # This is whether we ignore whether the system is armed or not and performs the data logging
check_once = 0
bad_data = 0
#########################################
Mavlink = MavConnection()
Log = FullLog()
heartBeatArmed = 0
initialize_items = 0
# Main Code
while True:
    start_time = time.time()
    while Mavlink.heartBeatArmed != 129 and test_mode == 0:
        print("Disarmed")
        # Since we are disarmed, we are going to reset the active sensors and create a new log file name that is just going to be the current time
        # That way when we arm up again it will make a new file.
        # log_file = datetime.now().strftime('%Y%m%d_%H%M%S') + ".txt"
        logged_data = ["time(s"]  # Array for Data that will be logged
        initialize_items = 0
        time.sleep(.1)
        Mavlink.get_message()

    while Mavlink.heartBeatArmed == 129 or test_mode == 1:
        if initialize_items == 0:
            Mavlink = MavConnection()
            Log.initialize_log_file()
            distanceInitialize = 0
            check_global_pos_int = 0
            gas_sense = CCS811()
            tempSense = Tmp117_Sensor()
            distSense = VL53l1x_distance_sensor()
            envSense = bme280_sensor()
            print("Data Being Logged: ")
            print(logged_data)
            print("\n")
            Log.log_data_file(logged_data)
            x = threading.Thread(target=camera_record)
            x.start()
            initialize_items = 1

        data = []
        data.append(round(time.time() - start_time, 3))
        # CCS811 Data
        gas_sense.read_gas()
        # Get TMP117 Data
        temperature = tempSense.tempReading()
        # Get VL53L1X Data
        distance = distSense.measure_distance()
        # Get BME280 and CCS811 data
        envSense.get_bme280_data()
        # Mavlink Get Message
        Mavlink.get_message()
        Mavlink.append_data()

        try:
            print(data)
        except:
            pass
        Log.log_data_file(data)
        # time.sleep(0.1)
