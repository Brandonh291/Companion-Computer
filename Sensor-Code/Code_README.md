



#How to create service:

# How to enable service:
The service's name is sensorPackage.service

To START use: "sudo systemctl enable sensorPackage.service"

To STOP  use: "sudo systemctl stop sensorPackage.service"


# How to make commits back to the git page:

To commit this back to the git page
git add SensorPackage.py
git commit -m "COMMENT"
git push origin main

# How to update?
git pull origin main


# List of things you need Installed
- VL53L1X: sudo pip install VL53L1X
- smbus2: sudo pip install smbus2
- bme280: sudo pip install pimoroni-bme280
- pymavlink: sudo python -m pip install --upgrade pymavlink

# Current Notes to Work On:
- I would like to implement a method of running the sensor package without needing a connection to a Mavlink Device. If I can do an attempted connection and separate the routine of grabbing data from Mavlink as being a requirement of running the other sensor measurements then I could add this as a functionality of being able to just make this a sand-alone device that you can attach to just any sort of vehicle for collecting data.
-  
