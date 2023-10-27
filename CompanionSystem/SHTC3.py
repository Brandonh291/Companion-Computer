# SHTC3 Sensor
# Sparkfun Breakout Board
from smbus2 import SMBus                                        # I2C Package
from smbus2 import i2c_msg                                      # I2C Package

class SHTC3:
    def __init__(self,busID):
        # Attempt to initialize device
        try:
            self.busID = busID
            self.bus=SMBus(self.busID)
            self.addr = 0x70                                      # Device Address
            self.wakeupMSB = 0x35                                 # MSB of Wakeup Command
            self.wakeupLSB = 0x17                                 # LSB of Wakeup Command
            self.sleepMSB = 0xb0                                  # MSB of Sleep Command
            self.sleepLSB = 0x98                                  # LSB of Sleep Command
            self.temp = 0
            self.humidity=0
            
            msg=i2c_msg.write(0x70,[0x35,0x17])                 # Write Wakeup Command
            self.bus.i2c_rdwr(msg)                                   # Send Wakeup Command
            msg=i2c_msg.write(0x70,[0b01011100,0b00100100])     # Write Measure Command
            self.bus.i2c_rdwr(msg)                                   # Send Measure Command
            msg=i2c_msg.read(0x70,6)                            # Write Read Command
            self.bus.i2c_rdwr(msg)                                   # Send Read Command
            
            self._running = True                                # Initialization Succeeded
                                                                # System will take data from sensor.
            print("SHTC3 Pass")                                 # Let user know the sensor passed initialization.
            
        # Failure to initialize device.
        except:
            self._running = False                               # Initialization Failed.
                                                                # System will not take data from sensor.
            print("SHTC3 Fail")                                 # Let user know the sensor failed initialization.
            
    # Read Temperature and Humidity Data    
    def measure(self):
        if self._running:
            msg = i2c_msg.write(0x70,[0x35,0x17])                 # Write Wakeup Command
            self.bus.i2c_rdwr(msg)                                   # Send Wakeup Command
            msg = i2c_msg.write(0x70,[0b01011100,0b00100100])     # Write Measure Command
            self.bus.i2c_rdwr(msg)                                   # Send Measure Command
            msg = i2c_msg.read(0x70,6)                            # Write Read Command
            self.bus.i2c_rdwr(msg)                                   # Send Read Command
            
            data = list(msg)                                      # Load data from command into an array

            self.temp = data[3]<<8|data[4]                        # Combine Temperature Bytes
            self.temp = (-45)+((175*self.temp)/65536)             # Convert to Celsius Value

            self.humidity = data[0]<<8|data[1]                    # Combine Humidity Bytes
            self.humidity = 100*self.humidity/65536               # Convert to % Humidity
      