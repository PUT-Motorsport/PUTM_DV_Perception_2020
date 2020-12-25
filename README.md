# PUTM_DV_Perception_2020
PUT Motorsport Driverless perception systems with use of Lidar, AHRS and GPS sensors.

<!-- ## LiDAR sensor
### Sensor information

### ROS package

#### Instalation

#### Usage


### Sensor data
#### Topics

#### Messages


## AHRS sensor
### Sensor information

### ROS package

#### Instalation

#### Usage


### Sensor data
#### Topics

#### Messages -->



## GPS sensor
### Sensor information
[Grove GPS sensor](https://www.seeedstudio.com/Grove-GPS-Module.html) with comunication via UART and USB UART converter [OKY3412](https://www.okystar.com/product-item/pl2303-pl2303hx-usb-to-rs232-oky3412/).

### ROS package
GPS message package: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver).

#### Instalation
ROS noetic:
```bash
sudo apt install ros-noetic-nmea-navsat-driver
```

#### Usage
```bash
roslaunch nmea_navsat_driver nmea_serial_driver.launch port:=/dev/ttyUSB0 baud:=9600
```

### Sensor data
#### Topics
```bash
/fix                # GPS data
/heading
/time_reference
/vel                # GPS velocity
```
#### Messages
```bash
header: 
  seq: 47
  stamp: 
    secs: 1608914690
    nsecs:         0
  frame_id: "gps"
status: 
  status: 0
  service: 1
latitude: 52.13249166666667
longitude: 16.129443333333334
altitude: nan
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0
```