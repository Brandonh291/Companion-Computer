# CCS811 Environmental Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
import time

# Constants
DEVICE_ADDRESS = 0x5B
STATUS = 0x00
MEAS_MODE = 0x01
ALG_RESULT_DATA = 0x02
RAW_DATA = 0x03
ENV_DATA = 0x05
THRESHOLDS = 0x10
BASELINE = 0x11
HW_ID = 0x20
HW_VERSION = 0x21
FW_BOOT_VERSION = 0x23
FW_APP_VERSION = 0x24
INTERNAL_STATE = 0xA0
ERROR_ID = 0xE0
APP_ERASE = 0xF1
APP_DATA = 0xF2
APP_VERIFY = 0xF3
APP_START = 0xF4
SW_RESET = 0xFF

class CCS811:
    """
    CCS811 Environmental Sensor class for Sparkfun Breakout Board.

    Parameters:
    - busID (int): The bus ID for communication with the sensor.

    Attributes:
    - busID (int): The bus ID for communication.
    - bus (SMBus): The SMBus instance for communication.
    - address (int): Device address (default: 0x5B).

    - eco2 (int): CO2 concentration in parts per million (ppm).
    - eTVOC (int): Total Volatile Organic Compounds concentration in parts per billion (ppb).
    - dataReady (bool): Indicates whether data is ready for reading.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self, busID): Initialize the CCS811 sensor.
    - checkStatus(self): Check if data is available for reading.
    - read_gas(self): Read gas concentration values from the sensor.
    """

    def __init__(self, busID=3, address = DEVICE_ADDRESS):
        """
        Initialize the CCS811 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        - address (int): The address for the sensor.
        """
        try:
            self.busID = busID
            self.bus = SMBus(self.busID)
            self.address = address  # Device Address

            self.eco2 = 0  # CO2 in parts per million (ppm)
            self.eTVOC = 0  # Total Volatile Organic Compounds in parts per billion (ppb)
            self.dataReady = False
            self.configure()
            self._running = True
            print("CCS811 Passed")
        except:
            self._running = False
            print("CCS811 Failed")

    def configure(self, drive_mode = 0b001):
        """
        Start the device, configure the measurement mode, then clear first
        few readings.

        Parameters:
        - drive_mode (int): Sets the measurement mode and timing.
            0b000: Mode 0 – Idle (Measurements are disabled in this mode)
            0b001: Mode 1 – Constant power mode, IAQ measurement every second
            0b010: Mode 2 – Pulse heating mode IAQ measurement every 10 seconds
            0b011: Mode 3 – Low power pulse heating mode IAQ measurement every
                60 seconds
            0b100: Mode 4 – Constant power mode, sensor measurement every 250ms
            0b1xx: Reserved modes (For future use)
            In mode 4, the ALG_RESULT_DATA is not updated, only RAW_DATA;
                the processing must be done on the host system.
        """
        self.bus.write_byte(self.address, APP_START)
        self.bus.write_byte(self.address, MEAS_MODE)
        self.bus.write_i2c_block_data(self.address, MEAS_MODE, [drive_mode<<4])
        
        while self.eco2 == 0:
            self.bus.write_byte(self.address, ALG_RESULT_DATA)
            val = self.bus.read_i2c_block_data(self.address, ALG_RESULT_DATA, 4)
            self.eco2 = ((val[0] << 8) | (val[1]))
            time.sleep(.01)
            
    def checkStatus(self):
        """
        Check if data is available for reading.
        """
        if self._running:
            self.bus.write_byte(self.address, STATUS)
            val = self.bus.read_i2c_block_data(self.address, STATUS, 1)
            if val[0] & 0b00001000:
                self.dataReady = True
            else:
                self.dataReady = False

    def read_gas(self):
        """
        Read gas concentration values from the sensor.
        """
        if self._running:
            self.checkStatus()
            try:
                if self.dataReady:
                    self.bus.write_byte(self.address, ALG_RESULT_DATA)
                    time.sleep(0.005)
                    val = self.bus.read_i2c_block_data(self.address, ALG_RESULT_DATA, 4)
                    self.eco2 = ((val[0] << 8) | (val[1]))
                    self.eTVOC = ((val[2] << 8) | (val[3]))
            except:
                print("Error in Gas Read")
c=CCS811()
c.read_gas()