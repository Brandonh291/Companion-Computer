import os
from picamera import PiCamera
from picamera import Color
import threading
import VL53L1X
import time
from smbus2 import SMBus
from smbus2 import i2c_msg
from datetime import datetime
from bme280 import BME280
from pymavlink import mavutil
class Vehicle(threading.Thread):
    def __init__(self):
        try:
            threading.Thread.__init__(self)
            self.nav = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
            print("Waiting for Heartbeat") # We need to receive a correct signal from the controller to indicate we are atleast receiving data
        
            self.heartVal = self.nav.wait_heartbeat(timeout=10) # Waiting 10 seconds for a heart message
        
            if self.heartVal == None:
                print("Failure")
                self._running = False # If we receive no message, then set vehicle to not running so we dont mess up asking for additonal commands
            
            else:
                self._running = True
                print("Heartbeat from system: ",self.nav.
                      target_system," and component: ", self.nav.target_component)
                print(self.heartVal)
                self.setIntervals()
                self.msgRec=0
        except:
            self._running = False
            print("Mavlink Failed")
            
    def set_Message_Interval(self,message_id, frequency_Hz):
        if self._running:
            self.nav.mav.command_long_send(1,self.nav.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                       0,
                                       message_id,
                                       1e6/frequency_Hz,
                                       0,0,0,0,
                                       0)
            time.sleep(0.1)
    def getMessage(self):
        if self._running:
            self.msg=self.nav.recv_match().to_dict()
    def setIntervals(self):
        self.nav.mav.request_data_stream_send(1,0,mavutil.mavlink.MAV_DATA_STREAM_ALL,6,0)
        print("All Stream Halted")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,2) # VFR_HUD, 2Hz
        print("VFR HUD Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,2) #GPS_RAW_INT, 2 Hz
        print("POSITION_TARGET_GLOBAL_INT Stream Started") 
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS,2)
        print("AHRS Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,2)
        print("GPS_RAW_INT Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,2)
        print("RC_CHANNELS Stream Started") 
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,2)
        print("MISSION_CURRENT Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,2)
        print("NAV_CONTROLLER_OUTPUT Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,2)
        print("SYS_STATUS Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,2)
        print("GLOBAL_POSITION_INT Stream Started")
        self.set_Message_Interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,2)
        print("RAW_IMU Stream Started")
    def parseMessage(self):
        if self._running:
            if self.msg['mavpackettype']=='VFR_HUD':
                print(self.msg)
                self.VFR_HUD=self.msg
            elif self.msg['mavpackettype']=='POSITION_TARGET_GLOBAL_INT':
                print(self.msg)
                self.POS_TAR_GLOB_INT=self.msg
            elif self.msg['mavpackettype']=='AHRS':
                print(self.msg)
                self.AHRS=self.msg
            elif self.msg['mavpackettype']=='GPS_RAW_INT':
                print(self.msg)
                self.GPS_RAW_INT=self.msg
            elif self.msg['mavpackettype']=='RC_CHANNELS':
                print(self.msg)
                self.RC_CHANNELS=self.msg
            elif self.msg['mavpackettype']=='MISSION_CURRENT':
                print(self.msg)
                self.MISSION_CURRENT=self.msg
            elif self.msg['mavpackettype']=='NAV_CONTROLLER_OUTPUT':
                print(self.msg)
                self.NAV_CONTR_OUT=self.msg
            elif self.msg['mavpackettype']=='SYS_STATUS':
                print(self.msg)
                self.SYS_STATUS=self.msg
            elif self.msg['mavpackettype']=='GLOBAL_POSITION_INT':
                print(self.msg)
                self.GLOBAL_POS_INT=self.msg
            elif self.msg['mavpackettype']=='RAW_IMU':
                print(self.msg)
                self.RAW_IMU=self.msg
            else:
                pass
    def run(self):
        if self._running:
            while self._running:
                time.sleep(.05)
                #print(navio.nav.messages)
                try:
                    self.getMessage()
                    self.parseMessage()
                except:
                    print("passing")
                    pass

                
navio=Vehicle()



navio.start()
time.sleep(20)
navio._running=False
