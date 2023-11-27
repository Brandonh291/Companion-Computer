# CCS811 Environmental Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

class CCS811:
    """
    CCS811 Environmental Sensor class for Sparkfun Breakout Board.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - addr (int): Device address (default: 0x5B).
    - meas_mode (int): Measure Mode Register (default: 0x01).
    - app_start (int): Application Start Register (default: 0xF4).
    - alg_result (int): Algorithm Result Register (default: 0x02).
    - status_reg (int): Status Register (default: 0x00).
    - eco2 (int): CO2 concentration in parts per million (ppm).
    - eTVOC (int): Total Volatile Organic Compounds concentration in parts per billion (ppb).
    - dataReady (bool): Indicates whether data is ready for reading.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self, busID): Initialize the CCS811 sensor.
    - checkStatus(self): Check if data is available for reading.
    - read_gas(self): Read gas concentration values from the sensor.
    """

    def __init__(self, busID):
        """
        Initialize the CCS811 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        try:
            self.busID = busID
            self.bus = SMBus(self.busID)
            self.addr = 0x5B  # Device Address
            self.meas_mode = 0x01  # Measure Mode Register
            self.app_start = 0xF4  # Application Start Register
            self.alg_result = 0x02  # Algorithm Result Register
            self.status_reg = 0x00  # Status Register
            self.eco2 = 0  # CO2 in parts per million (ppm)
            self.eTVOC = 0  # Total Volatile Organic Compounds in parts per billion (ppb)
            self.dataReady = False
            self.bus.write_byte(self.addr, self.app_start)
            self.bus.write_byte(self.addr, self.meas_mode)
            self.bus.write_i2c_block_data(self.addr, self.meas_mode, [0b00010000])
            while self.eco2 == 0:
                self.bus.write_byte(self.addr, self.alg_result)
                val = self.bus.read_i2c_block_data(self.addr, self.alg_result, 4)
                self.eco2 = ((val[0] << 8) | (val[1]))
                self.eTVOC = ((val[2] << 8) | (val[3]))
                time.sleep(.01)
            self._running = True
            print("CCS811 Passed")
        except:
            self._running = False
            print("CCS811 Failed")

    def checkStatus(self):
        """
        Check if data is available for reading.
        """
        if self._running:
            self.bus.write_byte(self.addr, self.status_reg)
            val = self.bus.read_i2c_block_data(self.addr, self.status_reg, 1)
            if val[0] & 0b00001000:
                self.dataReady = True

    def read_gas(self):
        """
        Read gas concentration values from the sensor.
        """
        if self._running:
            self.checkStatus()
            try:
                if self.dataReady:
                    self.bus.write_byte(self.addr, self.alg_result)
                    time.sleep(0.005)
                    val = self.bus.read_i2c_block_data(0x5b, 0x02, 4)
                    self.eco2 = ((val[0] << 8) | (val[1]))
                    self.eTVOC = ((val[2] << 8) | (val[3]))
            except:
                print("Error in Gas Read")
