# TMP117 Temperature Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                                        # I2C Package
from smbus2 import i2c_msg                                                      # I2C Package    

# Constants   
DEVICE_ADDRESS =    0x48                                                        # Default Device Address      
TEMP_RESULT =       0x00                                                        # Temperature result register
CONFIGURATION =     0x01                                                        # Configuration register
THIGH_LIMIT =       0x02                                                        # Temperature high limit register
TLOW_LIMIT =        0x03                                                        # Temperature low limit register
TEMP_OFFSET =       0x07                                                        # Temperature offset register
DEVICE_ID =         0x0F                                                        # Device ID register

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
    - dataHere (bool): Flag indicating whether new data is available for 
                        reading.
    - temp_c (float): Temperature reading in Celsius.
    - _running (bool): Flag indicating whether the sensor initialization 
                        succeeded.
                        
    Methods:
    - __init__(self, busID): Constructor method for the TMP117 class.
    - configure(self, conversion_mode, conversion_cycle, conversion_average):
        Configures TMP117 Sensor.
    - twos_comp(self, val, bits): Calculate the two's complement of a value.
    - dataReady(self): Check if new data is available for reading.
    - tempReading(self): Read temperature data from the sensor and convert 
                        it to Celsius.
    """
    def __init__(self, busID = 3, address = DEVICE_ADDRESS):
        """
        Initialize the TMP117 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        try:
            self.busID=busID
            self.bus=SMBus(busID)
            self.address = address                                              # Device Address 
            
            self.dataHere = False                                               # Initialize Flag for whether there is data to be read
            self.temp_c = 0                                                     # Initialize data output......Removeable?
            
            self.bus.read_byte_data(self.address, TEMP_RESULT)                  # Read the temperature register.


            self.configure() # Configure Device
            self._running = True                                                # Initialization succeeded
                                                                                # System will take data from this sensor.
                                                        
            print("TMP117 Pass")                                                # Let the user know the sensor passed initialization.
            
        except:
            self._running = False                                               # Initialization failed, mark as non-functioning.
                                                                                # System will not take data from this sensor.
                                                        
            print("TMP117 Fail")                                                # Let the user know the sensor failed initialization.

    def configure(self, conversion_mode = 0b00, conversion_cycle = 0b001, 
                  conversion_average = 0b01):
        """
        Configure Sensor.

        Parameters:
        - conversion_mode (binary): Sets conversion Mode.
            0b00: Continuous conversion (CC)
            0b01: Shutdown (SD)
            0b10: Continuous conversion (CC), Same as 00 (reads back = 00)
            0b11: One-shot conversion (OS)
            
        - Conversion Cycle Bit (binary): Stanby Time between Conversions. There
            are a lot of option settings in here but we are focused on a cycle
            time of about 125 ms. More information available on datasheet.
            
        - Conversion Averaging Mode(binary): Determines the number of 
            conversion results that are colelcted and averaged before updating
            the temperature register.The average is an accumulated average and
            not a running average.
            0b00: No averaging
            0b01: 8 Averaged conversions
            0b10: 32 averaged conversions
            0b11: 64 averaged conversions
        """
        
        val=[0,1] # Create Register Variable
        val[1] = ((conversion_cycle & 0b001)<<7) \
            | (conversion_average<<5) | 0b00000                                 # LSB of Configuration Register                                                         
        val[0] = (conversion_mode<<2) | (conversion_cycle>>1)                   # MSB of Configuration Register
        
        self.bus.write_i2c_block_data(self.address,CONFIGURATION, val)          # Write the new configuration to the register.   
        
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
        val = self.bus.read_i2c_block_data(self.address,
                                           TEMP_RESULT, 2)           # Read the Configuration Register.
        
        if val[0] & 0b100000:                                                   # Check if Bit 13 of the configuration register is "1".
            self.dataHere = True                                                # If Bit 13 == 1, New data is available.   
        else:
            self.dataHere = False                                               # Else, new data is not available.

    def tempReading(self):
        """
        Read temperature data from the sensor and convert it to Celsius.

        If the sensor has been initialized successfully and new data is 
        available, read the temperature register, perform necessary bit 
        operations, and convert the result to Celsius.
        """
        if self._running:                                                       # Only reads data if system passed initialization.
            self.dataReady()                                                    # Check if data is available to be read. 
            if self.dataHere:                                                   # If data is available, read from temperature register.
                val = self.bus.read_i2c_block_data(self.address,
                                                   TEMP_RESULT, 2)     # Read temperature data register and store the recieved 2 bytes.                    
                self.temp_c = (val[0] << 8) | (val[1] >> 0)                     # Combine MSB and LSB bytes of the data.
                self.temp_c = self.twos_comp(self.temp_c, 16)                   # Calculate Twos complement of combined data.

                self.temp_c = self.temp_c * 0.0078125                           # Convert to celsius. 