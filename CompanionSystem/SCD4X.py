# SCD40 CO2, Temp, Humidity
# Adafruit Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

class SCD4X_CO2:
    """
    SCD4X_CO2 class for interfacing with the SCD4X CO2 sensor.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - address (int): Device address (default: 0x62).
    - CO2 (int): CO2 concentration data.
    - Temp (float): Temperature data in Celsius.
    - RH (float): Relative Humidity data in percentage.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self, busID): Initialize the SCD4X_CO2 sensor.
    - readSensor(self): Read CO2, temperature, and humidity data from the sensor.
    """

    def __init__(self, busID):
        """
        Initialize the SCD4X_CO2 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        self.busID = busID
        self.bus = SMBus(self.busID)
        self.address = 0x62
        time.sleep(1)  # Startup time
        self.bus.write_i2c_block_data(self.address, 0, [0x21, 0xb1])
        time.sleep(5)
        self.bus.write_i2c_block_data(self.address, 0, [0xEC, 0x05])
        time.sleep(0.05)
        b = self.bus.read_byte_data(self.address, 0)
        print(b)

        self.bus.write_byte(0x62, 0x21b1)  # Begin Periodic Measurements
        msg = i2c_msg.write(0x62, [0x21, 0xb1])
        self.bus.i2c_rdwr(msg)
        time.sleep(5)
        # self.bus.write_byte(0x62,0xec05)
        write = i2c_msg.write(0x62, [0xEC, 0x05])
        read = i2c_msg.read(0x62, 1)
        self.bus.i2c_rdwr(write, read)
        time.sleep(0.01)
        data = list(read)
        print(data)
        self.CO2 = data[0] << 8 or data[1]
        self.Temp = data[3] << 8 or data[4]
        self.RH = data[6] << 8 or data[7]
        self.Temp = -45.0 + 175 * self.Temp / 65536
        self.RH = 100.0 * self.RH / 65536
        self._running = True

    def readSensor(self):
        """
        Read CO2, temperature, and humidity data from the sensor.
        """
        if self._running:
            self.bus.write_byte(0x62, 0xec05)
            time.sleep(0.005)
            read = i2c_msg.read(0x62, 9)
            self.bus.i2c_rdwr(read)
            result = list(read)
            # print(data)
            self.CO2 = result[0] << 8 or result[1]
            self.Temp = result[3] << 8 or result[4]
            self.RH = result[6] << 8 or result[7]
            self.Temp = -45.0 + 175.0 * self.Temp / 65536.0
            self.RH = 100.0 * self.RH / 65536.0
