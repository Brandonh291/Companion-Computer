# SHTC3 Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package

# Constants
DEVICE_ADDRESS = 0x70
SLEEP = [0xB0, 0x98]
WAKEUP = [0x35, 0x17]
NORMAL_MODE_RH_FIRST_CLOCK_STRETCH = [0x5C, 0x24]

class SHTC3:
    """
    SHTC3 Temperature and Humidity Sensor class.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.
    - address (int): The Address of the sensor

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - addr (int): Device address (default: 0x70).
    - temp (float): Temperature value in Celsius.
    - humidity (float): Humidity value in percentage.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self, busID): Initialize the SHTC3 sensor.
    - measure(self): Read temperature and humidity data from the sensor.
    """

    def __init__(self, busID = 3, address = DEVICE_ADDRESS):
        """
        Initialize the SHTC3 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        try:
            self.busID = busID
            self.bus = SMBus(self.busID)
            self.address = address

            self.temp = 0
            self.humidity = 0
            
            

            self._running = True  # Initialization Succeeded
            self.measure() 
            print("SHTC3 Pass")  # Let user know the sensor passed initialization.

        except:
            self._running = False  # Initialization Failed.
            print("SHTC3 Fail")  # Let user know the sensor failed initialization.

    def measure(self):
        """
        Read temperature and humidity data from the sensor.
        """
        if self._running:
            msg = i2c_msg.write(self.address, WAKEUP)  # Write Wakeup Command
            self.bus.i2c_rdwr(msg)  # Send Wakeup Command
            msg = i2c_msg.write(self.address, NORMAL_MODE_RH_FIRST_CLOCK_STRETCH)  # Write Measure Command
            self.bus.i2c_rdwr(msg)  # Send Measure Command
            msg = i2c_msg.read(self.address, 6)  # Write Read Command
            self.bus.i2c_rdwr(msg)  # Send Read Command

            data = list(msg)  # Load data from command into an array

            self.temp = data[3] << 8 | data[4]  # Combine Temperature Bytes
            self.temp = (-45) + ((175 * self.temp) / 65536)  # Convert to Celsius Value

            self.humidity = data[0] << 8 | data[1]  # Combine Humidity Bytes
            self.humidity = 100 * self.humidity / 65536  # Convert to % Humidity
x=SHTC3()