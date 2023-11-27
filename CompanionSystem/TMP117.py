# TMP117 Temperature Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package                

class TMP117:
    """
    Class representing a TMP117 temperature sensor.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus object for communication.
    - tmp117_addr (int): Device address for the TMP117 sensor.
    - tmp117_reg_temp (int): Register address for temperature readings.
    - tmp117_reg_config (int): Register address for configuration settings.
    - dataHere (bool): Flag indicating whether new data is available for reading.
    - temp_c (float): Temperature reading in Celsius.
    - _running (bool): Flag indicating whether the sensor initialization succeeded.

    Methods:
    - __init__(self, busID): Constructor method for the TMP117 class.
    - twos_comp(self, val, bits): Calculate the two's complement of a value.
    - dataReady(self): Check if new data is available for reading.
    - tempReading(self): Read temperature data from the sensor and convert it to Celsius.
    """
    def __init__(self, busID):
        """
        Initialize the TMP117 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
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

    def twos_comp(self, val, bits):
        """
        Calculate the two's complement of a value.

        Parameters:
        - val (int): The value to calculate the two's complement for.
        - bits (int): The number of bits in the representation.

        Returns:
        - int: The two's complement of the input value.
        """
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def dataReady(self):
        """
        Check if new data is available for reading.

        Updates the dataHere attribute based on the configuration register.
        """
        val = self.bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_config, 2)         # Read the Configuration Register.
        
        if val[0] & 0b100000:                                    # Check if Bit 13 of the configuration register is "1".
            self.dataHere = True                                # If Bit 13 == 1, New data is available.   
        else:
            self.dataHere = False                               # Else, new data is not available.

    def tempReading(self):
        """
        Read temperature data from the sensor and convert it to Celsius.

        If the sensor has been initialized successfully and new data is available, read the temperature register,
        perform necessary bit operations, and convert the result to Celsius.
        """
        if self._running == 1:                                    # Only reads data if system passed initialization.
            self.dataReady()                                    # Check if data is available to be read. 
            if self.dataHere:                                   # If data is available, read from temperature register.
                val = self.bus.read_i2c_block_data(self.tmp117_addr,self.tmp117_reg_temp, 2) # Read temperature data register and store the recieved 2 bytes.                    
                self.temp_c = (val[0] << 8) | (val[1] >> 0)     # Combine MSB and LSB bytes of the data.
                self.temp_c = self.twos_comp(self.temp_c, 16)   # Calculate Twos complement of combined data.

                self.temp_c = self.temp_c * 0.0078125           # Convert to celsius. 

