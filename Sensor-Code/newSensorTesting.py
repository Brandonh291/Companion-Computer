# Libraries
import time
#from threading import Thread
#try sudo pip install vl53l1x
from smbus2 import SMBus, i2c_msg
from enum import Enum
#import qwiic_vl53l1x
import sys
from datetime import datetime
from math import log10, floor
from pymavlink import mavutil
ccs811_addr=0x5B
bme280_addr=0x77
global eco2
from bme280 import BME280
import VL53L1X
bus = SMBus(4)
logged_data=[]
data=[]
class microPressure:
    def __init__(self):
        try:
            global logged_data
            #address is 0x18
            data1=[]
            msg=i2c_msg.write(0x18,[0xAA,0X00,0x00])
            data1=list(msg)
            bus.i2c_rdwr(msg)
            time.sleep(1)
            val=bus.read_byte(0x18)
            print(val)
            msg=i2c_msg.read(0x18,4)
            bus.i2c_rdwr(msg)
            data1=list(msg)
            status=data1[0]
            print(bin(status))
            self.Pressure=0
            self._running=True
            logged_data.append("Micro Pressure Sensor Pressure (psi)")
            #print("Did I get Here")
        except:
            self._running=False
            print("Exception in Initialization")
    def read_pressure(self):
        try:
            if self._running:
                global data
                #address is 0x18
                cont=[]
                msg=i2c_msg.write(0x18,[0xAA,0X00,0x00])
                cont=list(msg)
                bus.i2c_rdwr(msg)
                time.sleep(1)
                val=bus.read_byte(0x18)
                #print(val)
                msg=i2c_msg.read(0x18,4)
                bus.i2c_rdwr(msg)
                cont=list(msg)
                outputs=cont[1]<<16 or cont[2]<<8 or cont[3]
                #print(outputs)
                # 10% to 90% calibration
                output_max=0xE66666
                output_min=0x19999A
                Pmax=25.000 # max psi
                Pmin=0.000 # min psi
                #print("Calc")
                
                self.Pressure=(outputs-output_min)*(Pmax-Pmin)
                self.Pressure=(self.Pressure / (output_max-output_min))+Pmin
                print(self.Pressure)
                data.append(self.Pressure)              
        except:
            print("Failed Pressure Reading")
def read_distance():
    tof=VL53L1X.VL53L1X(i2c_bus=4,i2c_address=0x29)
    tof.open()
    tof.start_ranging(1)
    distance=tof.get_distance()
    print(distance)
    tof.stop_ranging()
    
def read_humidity_press():
    for i in range(30):
        bus=SMBus(4)
        bme280 = BME280(i2c_dev=bus,i2c_addr=0x77)
        temperature = bme280.get_temperature()
        pressure = bme280.get_pressure()
        humidity = bme280.get_humidity()
        altitude=bme280.get_altitude()
        print("Altitude: ",altitude)
        print('{:05.2f}*C {:05.2f}hPa {:05.2f}%'.format(temperature, pressure, humidity))
        #mySensor1.set_environmental_data(humidity, temperature)
        time.sleep(1)
def read_gas():
    global eco2
    global eTVOC
    # Will need to wait 20 minutes for decent readings
    bus = SMBus(4)
    
    # Done at Startup, exit boot
    bus.write_byte(0x5B,0xF4)
    
    #Set Mode
    bus.write_byte(0x5B,0x01)
    bus.write_i2c_block_data(0x5B,0x01,[0b00010000])
    for i in range(10):
        # Go to Data
        #Ignore first 3, they are calibrations
        bus.write_byte(0x5B,0x02)
        val=bus.read_i2c_block_data(0x5b,0x02,4)
        eco2=((val[0]<<8)|(val[1]))
        eTVOC=((val[2]<<8)|(val[3]))
        print("eCO2: "+ str(eco2) + " ppm")
        print("eTVOC: "+ str(eTVOC) + "ppb")
        time.sleep(1)
    
    
def print_gas():
    time.sleep(2)
    print(eco2)
    
class SCD4X_CO2:
    
    def __init__(self):
        global logged_data
        try:
            self.address=0x62
            time.sleep(1) # Startup time
            bus.write_byte(0x62,0x21b1) # Begin Periodic Measurements
            time.sleep(0.1)
            bus.write_byte(0x62,0xec05)
            time.sleep(10)
            read=i2c_msg.read(0x62,9)
            bus.i2c_rdwr(read)
            data=list(read)
            #print(data)
            self.CO2=data[0]<<8 or data[1]
            self.Temp=data[3]<<8 or data[4]
            self.RH=data[6]<<8 or data[7]
            self.Temp=-45.0 + 175*self.Temp/65536
            self.RH=100.0*self.RH/65536
            print("CO2: " +str(CO2))
            print("Temp: " +str(Temp))
            print("RH: "+str(RH))
            logged_data.append("SCD4X CO2 PPM")
            logged_data.append("SCD4X Temperature (C)")
            logged_data.append("SCD4X Relative Humidity %")
            self._running=True
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
def read_temp():
    # Initialize I2C (SMBus)
    bus = SMBus(1)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    val[1] = val[1] & 0b00111111
    val[1] = val[1] | (0b10 << 6)
    bus.write_i2c_block_data(tmp117_addr, tmp117_reg_config, val)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_config, 2)
    val = bus.read_i2c_block_data(tmp117_addr, tmp117_reg_temp, 2)
    temp_c = (val[0] << 8) | (val[1] >> 0)
    temp_c = twos_comp(temp_c, 16)
    # Convert registers value to temperature (C)
    temp_c = temp_c * 0.0078125
    bus.close()
    return temp_c


class SHTC3:
    def __init__(self):
        #  # Keep an eye on this, I might need to do a general call for the Bus to the entire program
        global logged_data
        # try activating the CCS811 Sensor
        try:
            # Address is 0x70
            #Write Wakeup Command
            bus.write_i2c_block_data(0x70,0x35,[0x17])
            # Write Reah RH First with Clock Stretching
            bus.write_i2c_block_data(0x70, 0x5C,[0x24])
            val=bus.read_i2c_block_data(0x70,0x00,6)
            print(val)
            self._running = True

        except:
            print("SHTC3 not Detected")
            self._running = False
            return
        
class ADC():
    def __init__(self):
        self.addr=0x48
        write=bus.write_byte_data(self.addr,0x40,0xff)
        #read=i2c_msg.read(self.addr,3)
        #bus.i2c_rdwr(read)
        #for value in read:
             #print(value*(3.3/255.0))
#         write=i2c_msg.write(addr,[0x00])
#         #write=i2c_msg.write(addr,[0x44])
#         time.sleep(0.1)
#         
#         bus.i2c_rdwr(write)
#         read=i2c_msg.read(addr,3)
#         bus.i2c_rdwr(read)
#         for value in read:
#             print(value)
        #bus.write_i2c_block_data(addr,0b0000000,0)
        #bus.read_byte_data(addr,0)
        self.expectedValue=0
        self.ave=0
        self.maxDiff=0
        
    def read_ADC(self,channel):
        write=i2c_msg.write(self.addr,[64+channel])
        #read=i2c_msg.read(self.addr,channel)
        ave=0
        readings=5
        for i in range(readings):
            
            read=i2c_msg.read(self.addr,3)
            bus.i2c_rdwr(write,read)
            data=list(read)
            #print(data[2]*3.3/255.0)
            ave=ave+data[2]*3.2/256.0
            time.sleep(0.01)
        ave=ave/readings
        print("Average Voltage: "+str(ave))
        self.ave=ave
        print("Difference: " +str(self.expectedValue-self.ave))
        diff=self.expectedValue-self.ave
        if diff>self.maxDiff:
            self.maxDiff=diff
        
    def set_DAC(self,value):
        write=bus.write_byte_data(self.addr,0x40,value)
        self.expectedValue=value*0.012566
        print("Expected Value: " +str(value*0.012566))
        #read=i2c_msg.read(self.addr,2)
        #bus.i2c_rdwr(write,read)
        time.sleep(0.1)
#threadIt=Thread(target=print_gas)
#threadIt.start()
#read_distance()
#read_gas()
#hreadIt.terminate()
#read_humidity_press()
#he=microPressure()

#sht=SHTC3()
#he.read_pressure()
ad=ADC()
t=0
while t<5:
    ad.set_DAC(t)
    ad.read_ADC(0)
    t=t+1
print(ad.maxDiff)
sens=SCD4X_CO2()