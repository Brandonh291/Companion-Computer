# MPR Series - MPRLS0025PA00001A Micro pressure Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                                        # I2C Package
from smbus2 import i2c_msg                                                      # I2C Package
import time

#Constants
DEVICE_ADDRESS =    0x18
OUTPUT_COMMAND =    [0xAA , 0x00, 0x00]

class microPressure:
    """
    microPressure class for interfacing with the microPressure sensor.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - address (int): Device address (default: 0x18).

    - Pressure (float): Pressure data in psi.
    - _running (bool): Indicates whether the sensor is successfully 
        initialized.

    Methods:
    - __init__(self, busID): Initialize the microPressure sensor.
    - read_pressure(self): Read pressure data from the sensor.
    """

    def __init__(self, busID = 3, address = DEVICE_ADDRESS):
        """
        Initialize the microPressure sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        - address (int): The address for the sensor.
        """
        try:
            self.busID =    busID
            self.bus =      SMBus(self.busID)
            self.address =  address  

            self.Pressure = 0  

            self._running = True  
            print("Micropressure Sensor Passed") 
        except:
            self._running = False  
            print("Micropressure Sensor Failed") 

    def read_pressure(self):
        """
        Read pressure data from the sensor.
        """
        if self._running:
            msg = i2c_msg.write(self.address, OUTPUT_COMMAND)                   # Write a message to receive data.
            self.bus.i2c_rdwr(msg)                                              # Send Message

            time.sleep(0.01) 
            val = self.bus.read_byte(self.address)                              # Read Address

            msg = i2c_msg.read(self.address, 4)                                 # Write message to read 4 bytes from device
            self.bus.i2c_rdwr(msg)                                              # Send Message and receive 4 bytes
            content = list(msg)                                                 # Convert message to array
            outputs = (content[1] << 16) | (content[2] << 8) | content[3]       # Combine Data Bytes

            # 10% to 90% calibration
            output_max = 0xE66666
            output_min = 0x19999A
            Pmax = 25.000                                                       # max psi
            Pmin = 0.000                                                        # min psi

            self.Pressure = (outputs - output_min) * (Pmax - Pmin)              # Pressure Calculation
            self.Pressure = (self.Pressure / (output_max - output_min)) + Pmin  # Pressure Calculation