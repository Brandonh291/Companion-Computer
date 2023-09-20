# MPR Series - MPRLS0025PA00001A Micro pressure Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

class microPressure:
    def __init__(self,busID):
        # Attempt to initialize device
        try:
            self.busID = busID
            self.bus=SMBus(self.busID)
            self.addr = 0x18                                        # Device Address
            self.output_reg = 0xAA                                  # Output Command Byte
            self.data_reg = 0x00                                    # Command Byte
            self.Pressure = 0                                       # Pressure Data 
            
            msg=i2c_msg.write(self.addr,[self.output_reg,self.data_reg,self.data_reg]) # Write a message to receive data.
            self.bus.i2c_rdwr(msg)                                       # Send Message
            
            time.sleep(.1)                                          # Wait 10ms
            
            val=self.bus.read_byte(self.addr)                            # Write Address
            
            msg=i2c_msg.read(self.addr,4)                           # Write message to read 4 bytes from device
            self.bus.i2c_rdwr(msg)                                       # Send Message and receive 4 bytes
            
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
            self.bus.i2c_rdwr(msg)                                       # Send Message
            
            time.sleep(.01)                                          # Wait 10ms
            val=self.bus.read_byte(self.addr)                            # Write Address
            
            msg=i2c_msg.read(self.addr,4)                           # Write message to read 4 bytes from device
            self.bus.i2c_rdwr(msg)                                       # Send Message and receive 4 bytes
            content=list(msg)                                       # Convert message to array
            outputs=content[1]<<16 or content[2]<<8 or content[3]   # Combine Data Bytes

            # 10% to 90% calibration
            output_max=0xE66666
            output_min=0x19999A
            Pmax=25.000                                             # max psi
            Pmin=0.000                                              # min psi
            
            self.Pressure=(outputs-output_min)*(Pmax-Pmin)          # Pressure Calculation
            self.Pressure=(self.Pressure / (output_max-output_min))+Pmin # Pressure Calculation
