# CCS811 Environmental Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

class CCS811:
    def __init__(self,busID):
        # Attempt to initialize device 
        try:
            self.busID = busID
            self.bus=SMBus(self.busID)
            self.addr = 0x5B                                    # Device Address
            self.meas_mode = 0x01                               # Measure Mode Register
            self.app_start = 0xF4                               # Application Start Register
            self.alg_result = 0x02                              # Algorithm Result Register
            self.status_reg = 0x00 # status register
            
            self.eco2 = 0                                       # CO2 in parts per million (ppm)
            self.eTVOC = 0                                      # Total Volatile Organic Compounds in parts per billion (ppb)
            self.dataReady=False
            
            self.bus.write_byte(self.addr, self.app_start)           # Transition from boot to application start.
            self.bus.write_byte(self.addr, self.meas_mode)           # Select Measure Mode Register
            
            self.bus.write_i2c_block_data(self.addr,self.meas_mode, [0b00010000])          # Measurements once a second

            # Read data till no longer zero (should provide a minimum of 400ppm)
            while self.eco2 == 0:
                self.bus.write_byte(self.addr, self.alg_result)      # Change registers to algorithm result
            
                val = self.bus.read_i2c_block_data(self.addr,self.alg_result, 4)        # Read 4 bytes from algorithm result register
            
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
    # Check if Data is available
    def checkStatus(self):
        if self._running:
            self.bus.write_byte(self.addr, self.status_reg) # set to status register
            val = self.bus.read_i2c_block_data(self.addr,self.status_reg,1)
            if val[0] & 0b00001000:
                self.dataReady=True
    # Read Data
    def read_gas(self):
        if self._running:
            self.checkStatus()
            try:
                if self.dataReady:
                    self.bus.write_byte(self.addr, self.alg_result)          # Change registers to algorithm result
                    time.sleep(0.005)
                    val = self.bus.read_i2c_block_data(0x5b, 0x02, 4)        # Read 4 bytes from algorithm results register
                    self.eco2 = ((val[0] << 8) | (val[1]))              # Load first 2 bytes into CO2
                    self.eTVOC = ((val[2] << 8) | (val[3]))             # Load last 2 bytes into TVOC
            except:
                print("Error in Gas Read")
