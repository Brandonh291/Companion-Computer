# BME280 Environmental Sensor
# Sparkfun Breakout Board
from bme280 import BME280                                       # BME280 environmental sensor
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
bus=SMBus(3)
class BME280_Sensor:
    """
    BME280_Sensor class for interfacing with the BME280 sensor.

    Attributes:
    - bme280_sensor (BME280): BME280 instance for communication.
    - temperature (float): Temperature data in Celsius.
    - pressure (float): Pressure data in Pascal.
    - humidity (float): Humidity data in percentage.
    - altitude (float): Altitude data in meters.
    - _running (bool): Indicates whether the sensor is successfully initialized.

    Methods:
    - __init__(self): Initialize the BME280_Sensor.
    - get_bme280_data(self): Get temperature, pressure, humidity, and altitude data from the sensor.
    """

    def __init__(self):
        """
        Initialize the BME280_Sensor.
        """
        try:
            self.bme280_sensor = BME280(i2c_dev=bus, i2c_addr=0x77)  # Initialize BME280 Library
            self.bme280_sensor.get_temperature()  # Get Temperature to check if sensor is working
            self.temperature = 0
            self.pressure = 0
            self.humidity = 0
            self.altitude = 0
            self._running = True  # Initialization succeeded
            print("BME280 Passed")  # Let the user know the sensor passed initialization.
        except:
            self._running = False  # Initialization failed
            print("BME280 Failed")  # Let the user know the sensor failed initialization.

    def get_bme280_data(self):
        """
        Get temperature, pressure, humidity, and altitude data from the sensor.
        """
        if self._running:
            self.temperature = self.bme280_sensor.get_temperature()  # Get Temperature (C)
            self.pressure = 100 * self.bme280_sensor.get_pressure()  # Get Pressure (Pa)
            self.humidity = self.bme280_sensor.get_humidity()  # Get Humidity (%)
            self.altitude = self.bme280_sensor.get_altitude()  # Get Altitude (m)

