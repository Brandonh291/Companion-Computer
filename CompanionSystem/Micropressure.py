# MPR Series - MPRLS0025PA00001A Micro pressure Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

class microPressure:
    """
    microPressure class for interfacing with the microPressure sensor.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - addr (int): Device address (default: 0x18).
    - output_reg (int): Output Command Byte (default: 0xAA).
    - data_reg (int): Command Byte (default: 0x00).
    - Pressure (float): Pressure data in psi.
    - status (int): Sensor status information.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self, busID): Initialize the microPressure sensor.
    - read_pressure(self): Read pressure data from the sensor.
    """

    def __init__(self, busID):
        """
        Initialize the microPressure sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        try:
            self.busID = busID
            self.bus = SMBus(self.busID)
            self.addr = 0x18  # Device Address
            self.output_reg = 0xAA  # Output Command Byte
            self.data_reg = 0x00  # Command Byte
            self.Pressure = 0  # Pressure Data

            msg = i2c_msg.write(self.addr, [self.output_reg, self.data_reg, self.data_reg])  # Write a message to receive data.
            self.bus.i2c_rdwr(msg)  # Send Message

            time.sleep(0.1)  # Wait 100ms

            val = self.bus.read_byte(self.addr)  # Read Address

            msg = i2c_msg.read(self.addr, 4)  # Write message to read 4 bytes from device
            self.bus.i2c_rdwr(msg)  # Send Message and receive 4 bytes
            self.data = list(msg)  # Convert message to array
            self.status = self.data[0]  # Separate Status from data

            self._running = True  # Initialization succeeded
            print("Micropressure Sensor Passed")  # Let user know the sensor passed initialization.
        except:
            self._running = False  # Initialization failed
            print("Micropressure Sensor Failed")  # Let user know the sensor failed initialization.

    def read_pressure(self):
        """
        Read pressure data from the sensor.
        """
        if self._running:
            msg = i2c_msg.write(self.addr, [self.output_reg, self.data_reg, self.data_reg])  # Write a message to receive data.
            self.bus.i2c_rdwr(msg)  # Send Message

            time.sleep(0.01)  # Wait 10ms
            val = self.bus.read_byte(self.addr)  # Read Address

            msg = i2c_msg.read(self.addr, 4)  # Write message to read 4 bytes from device
            self.bus.i2c_rdwr(msg)  # Send Message and receive 4 bytes
            content = list(msg)  # Convert message to array
            outputs = (content[1] << 16) | (content[2] << 8) | content[3]  # Combine Data Bytes

            # 10% to 90% calibration
            output_max = 0xE66666
            output_min = 0x19999A
            Pmax = 25.000  # max psi
            Pmin = 0.000  # min psi

            self.Pressure = (outputs - output_min) * (Pmax - Pmin)  # Pressure Calculation
            self.Pressure = (self.Pressure / (output_max - output_min)) + Pmin  # Pressure Calculation

