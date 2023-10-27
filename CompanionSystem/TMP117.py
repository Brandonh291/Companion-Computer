# TMP117 Temperature Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package


class TMP117:
    def __init__(self,busID):
        # Attempt to initialize device
        try:
            self.busID=busID
            self.bus=SMBus(busID)
            self.tmp117_addr = 0x48                             # Device Address 
            self.tmp117_reg_temp = 0x00                         # Temperature Register Address
            self.tmp117_reg_config = 0x01                       # Configuration Register Address
            
            self.dataHere = False                               # Initialize Flag for whether there is data to be read
            self.temp_c = 0                                     # Initialize data output......Removeable?
            val=[0,1]
            self.bus.read_byte_data(self.tmp117_addr, 0x00)          # Read the temperature register.


            # Configure Sensor 
            val[1] = 0b10111100                                 # LSB of Configuration Register
            val[0] = 0b00000000                                 # MSB of Configuration Register
            self.bus.write_i2c_block_data(self.tmp117_addr,self.tmp117_reg_config, val)          # Write the new configuration to the register.                

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
        val = self.bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_config, 2)         # Read the Configuration Register.
        
        if val[0] & 0b100000:                                    # Check if Bit 13 of the configuration register is "1".
            self.dataHere = True                                # If Bit 13 == 1, New data is available.   
        else:
            self.dataHere = False                               # Else, new data is not available.
  
    # Read Data from Register
    def tempReading(self):
        if self._running == 1:                                    # Only reads data if system passed initialization.
            self.dataReady()                                    # Check if data is available to be read. 
            if self.dataHere:                                   # If data is available, read from temperature register.
                val = self.bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_temp, 2) # Read temperature data register and store the recieved 2 bytes.                    
                self.temp_c = (val[0] << 8) | (val[1] >> 0)     # Combine MSB and LSB bytes of the data.
                self.temp_c = self.twos_comp(self.temp_c, 16)   # Calculate Twos complement of combined data.

                self.temp_c = self.temp_c * 0.0078125           # Convert to celsius. 
