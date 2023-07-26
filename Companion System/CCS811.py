# CCS811 Environmental Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time
bus=SMBus(4)
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
