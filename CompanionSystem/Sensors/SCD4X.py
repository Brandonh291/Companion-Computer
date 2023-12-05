# SCD40 CO2, Temp, Humidity
# Adafruit Breakout Board
# Datasheet: https://sensirion.com/media/documents/48C4B7FB/64C134E7/Sensirion_SCD4x_Datasheet.pdf
# Github: https://github.com/Brandonh291/Masters-Project-for-Raspberry-Pi-Based-Companion-Computer/blob/main/CompanionSystem/Sensors/SCD4X.py
from smbus2 import SMBus                                                        # I2C Package
from smbus2 import i2c_msg                                                      # I2C Package
import time

# Constants
DEVICE_ADDRESS =                            0x62
START_PERIODIC_MEASUREMENT =                [0x21, 0xB1]
READ_MEASUREMENT =                          [0xEC, 0x05]
STOP_PERIODIC_MEASUREMENT =                 [0x3F, 0x86]
SET_TEMPERATURE_OFFSET =                    [0x24, 0x1D]
GET_TEMPERATURE_OFFSET =                    [0x23, 0x18]
SET_SENSOR_ALTITUDE =                       [0x24, 0x27]
GET_SENSOR_ALTITUDE =                       [0x23, 0x22]
SET_AMBIENT_PRESSURE =                      [0xE0, 0x00]
PERFORM_FORCED_RECALIBRATION =              [0x36, 0x2F]
SET_AUTOMATIC_SELF_CALIBRATION_ENABLED =    [0x24, 0x16]
GET_AUTOMATIC_SELF_CALIBRATION_ENABLED =    [0x23, 0x13]
START_LOW_POWER_PERIODIC_MEASUREMENT =      [0x21, 0xAC]
GET_DATA_READY_STATUS =                     [0xE4, 0xB8]
PERSIST_SETTINGS =                          [0x36, 0x15]
GET_SERIAL_NUMBER =                         [0x36, 0x82]
PERFORM_SELF_TEST =                         [0x36, 0x39]
PERFORM_FACTORY_RESET =                     [0x36, 0x32]
REINIT =                                    [0x36, 0x46]
MEASURE_SINGLE_SHOT =                       [0x21, 0x9D]
MEASURE_SINGLE_SHOT_RHT_ONLY =              [0x21, 0x96]

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
    - _running (bool): Indicates whether the sensor is successfully 
        initialized.

    Methods:
    - __init__(self, busID): Initialize the SCD4X_CO2 sensor.
    - startPeriodicMeasurement: Start measurement polling.
    - getReadyStatus: Read the GET_DATA_READY_STATUS Register to determine 
        if the first 11 bits of the first word are greater than 0.
    - readSensor(self): Read CO2, temperature, and humidity data from the 
        sensor.
    """

    def __init__(self, busID=3,address=DEVICE_ADDRESS):
        """
        Initialize the SCD4X_CO2 sensor.

        Parameters:
        - busID (int): The bus ID for communication with the sensor.
        """
        self.busID =    busID
        self.bus =      SMBus(self.busID)
        self.address =  address
        
        time.sleep(1)  
        
        self.startPeriodicMeasurement()
        self._running = True

    def startPeriodicMeasurement(self):
        """
        Starts periodic measurements. The data will become available roughly 
        every 5 seconds.
        """
        msg = i2c_msg.write(self.address, START_PERIODIC_MEASUREMENT)
        self.bus.i2c_rdwr(msg)
        
    def getReadyStatus(self): 
        """
        Read the GET_DATA_READY_STATUS Register to determine if the first 11
        bits of the first word are greater than 0. If they are, that means
        that data is ready to be read.
        
        Returns:
        - bool: If the data is available to be read, then True.
        """
        msg = i2c_msg.write(self.address, GET_DATA_READY_STATUS)
        read = i2c_msg.read(self.address, 3)
        self.bus.i2c_rdwr(msg)
        
        data =          list(read)
        
        check=          data[0] << 8 or data[1]
        if (check & 0b0000011111111111) >0:
            flag =      True
            
        return flag
        
    def readSensor(self):
        """
        Read CO2, temperature, and humidity data from the sensor.
        """
        if self._running:
            dataAvailable = self.getReadyStatus()
            if dataAvailable: 
                
                write = i2c_msg.write(self.address, READ_MEASUREMENT)
                read = i2c_msg.read(self.address, 9)
                self.bus.i2c_rdwr(write, read)
                
                time.sleep(0.01)
                
                data =      list(read)

                self.CO2 =  data[0] << 8 or data[1]
                self.Temp = data[3] << 8 or data[4]
                self.RH =   data[6] << 8 or data[7]
                self.Temp = -45.0 + 175 * self.Temp / 65536
                self.RH =   100.0 * self.RH / 65536