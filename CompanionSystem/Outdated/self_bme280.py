# BME280 Environmental Sensor
# Sparkfun Breakout Board
from bme280 import BME280                                       # BME280 environmental sensor
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package
bus=SMBus(4)
class bme280_sensor:
    def __init__(self):
        # Attempt to initialize device
        try:
            self.bme280_sensor = BME280(i2c_dev=bus, i2c_addr=0x77) # Initialize BME280 Library
            self.bme280_sensor.get_temperature()                    # Get Temperature to check if sensor is working
            self.temperature = 0
            self.pressure = 0
            self.humidity = 0
            self.altitude = 0
            self._running = True                                    # Initialization succeeded
                                                                    # System will take data from sensor
            print("BME280 Passed")                                  # Let user know the sensor passed initialization.
        except:
            self._running = False                                   # Initialization failed
                                                                    # System will not take data from sensor
            print("BME280 Failed")                                  # Let user know the sensor failed initialization.
    
    # Get Device Data
    def get_bme280_data(self):
        if self._running:
            self.temperature = self.bme280_sensor.get_temperature() # Get Temperature (C)
            self.pressure = 100*self.bme280_sensor.get_pressure()   # Get Pressure (Pa)
            self.humidity = self.bme280_sensor.get_humidity()       # Get Humidity (%)
            self.altitude = self.bme280_sensor.get_altitude()       # Get Altitude (m)
