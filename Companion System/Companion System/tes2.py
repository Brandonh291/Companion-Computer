import time
from smbus2 import SMBus
from smbus2 import i2c_msg
bus=SMBus(4)
logged_data=[]
data=[]
class SCD4X_CO2:
    
    def __init__(self):
        global logged_data
        try:
            self.address=0x62
            time.sleep(1) # Startup time
            bus.write_byte(0x62,0x21b1) # Begin Periodic Measurements
            time.sleep(0.1)
            #bus.write_byte(0x62,0xec05)
            time.sleep(0.005)
            read=i2c_msg.read(0x62,9)
            bus.i2c_rdwr(read)
            data=list(read)
            #print(data)
            self.CO2=data[0]<<8 or data[1]
            self.Temp=data[3]<<8 or data[4]
            self.RH=data[6]<<8 or data[7]
            self.Temp=-45.0 + 175*self.Temp/65536
            self.RH=100.0*self.RH/65536
            #print("CO2: " +str(CO2))
            #print("Temp: " +str(Temp))
            #print("RH: "+str(RH))
            logged_data.append("SCD4X CO2 PPM")
            logged_data.append("SCD4X Temperature (C)")
            logged_data.append("SCD4X Relative Humidity %")
            self._running=True
            print("working?")
        except:
            self._running=False
            print("SCD4X Failure")
            
    def readSensor(self):
        global data
        if self._running:
            time.sleep(5)
            bus.write_byte(0x62,0xec05)
            time.sleep(0.005)
            read=i2c_msg.read(0x62,9)
            bus.i2c_rdwr(read)
            result=list(read)
            #print(data)
            self.CO2=result[0]<<8 or result[1]
            self.Temp=result[3]<<8 or result[4]
            self.RH=result[6]<<8 or result[7]
            self.Temp=-45.0 + 175.0*self.Temp/65536.0
            self.RH=100.0*self.RH/65536.0
            data.append(self.CO2)
            data.append(self.Temp)
            data.append(self.RH)

c=SCD4X_CO2()