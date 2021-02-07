# PUTM_DV_Perception_2020
PUT Motorsport Driverless perception systems with use of Lidar, AHRS and GPS sensors.

<!-- ## LiDAR sensor
### Sensor information

### ROS package

#### Instalation

#### Usage


### Sensor data
#### Topics

#### Messages -->


## AHRS sensor
### Sensor information
[Pololu UM7 Orientation Sensor](https://www.pololu.com/product/2764/resources) with comunication via UART and USB UART converter [OKY3412](https://www.okystar.com/product-item/pl2303-pl2303hx-usb-to-rs232-oky3412/).

### ROS package
Pololu UM7 AHRS sensor use ROS [um7](http://wiki.ros.org/um7) package to communication and need [serial](http://wiki.ros.org/serial) package to communicate with sensor.

#### Instalation
ROS noetic doesn't have package for serial and um7 repositories so necessary is to git clone repositories to catkin workspace build them.
```bash
cd ~/dv_ws/src/

git clone https://github.com/wjwwood/serial.git
git clone https://github.com/ros-drivers/um7.git
```

#### Usage
```bash
rosrun um7 um7_driver _port:=/dev/ttyUSB0
```

### Sensor data
#### Topics
```bash
/imu/data
/imu/mag
/imu/rpy
/imu/temperature
```
#### Messages
`/imu/data` topic example message
```
header: 
  seq: 1479
  stamp: 
    secs: 1609349210
    nsecs: 657628200
  frame_id: "imu_link"
orientation: 
  x: 0.004431147599999999
  y: 0.8119070897999999
  z: 0.5837029883999999
  w: 0.0035247764999999996
orientation_covariance: [0.002741552146694444, 0.0, 0.0, 0.0, 0.002741552146694444, 0.0, 0.0, 0.0, 0.007615422629706791]
angular_velocity: 
  x: 0.00023458541586861913
  y: 0.00035540586947785483
  z: -0.00027167016161300705
angular_velocity_covariance: [1.0966208586777776e-06, 0.0, 0.0, 0.0, 1.0966208586777776e-06, 0.0, 0.0, 0.0, 1.0966208586777776e-06]
linear_acceleration: 
  x: 0.13459433927363715
  y: -0.0012916299783646535
  z: 9.7126530349195
linear_acceleration_covariance: [0.0015387262937311438, 0.0, 0.0, 0.0, 0.0015387262937311438, 0.0, 0.0, 0.0, 0.0015387262937311438]
```


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


## LIDAR sensor
### Sensor information
[SICK MRS1104](https://www.sick.com/de/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044?ff_data=JmZmX2lkPXA0OTUwNDQmZmZfbWFzdGVySWQ9cDQ5NTA0NCZmZl90aXRsZT1NUlMxMTA0Qy0xMTEwMTEmZmZfcXVlcnk9JmZmX3Bvcz0xJmZmX29yaWdQb3M9MSZmZl9wYWdlPTEmZmZfcGFnZVNpemU9OCZmZl9vcmlnUGFnZVNpemU9OCZmZl9zaW1pPTkyLjA=) with communication via TCP/IP.

### ROS package
SICK MRS1104 sensor use ROS [sick_scan](http://wiki.ros.org/sick_scan) package to provide the measurement data as PointCloud2 and LaserScan data.

#### Instalation
ROS noetic:
```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone -b devel --single-branch git://github.com/SICKAG/sick_scan.git
cd ..
catkin_make
source ~/ros_catkin_ws/install/setup.bash
```


#### Usage
1. Use the [SICK SOPAS ET Software](https://www.sick.com/de/de/sopas-engineering-tool-2020/p/p367244) (working only on Windows 7,8,10) to determine or set the IP address of the laser scanner.
2. Change IP address is in the launch-file associated with your scanner type.
3. Launch the node e.g.
```bash
roslaunch sick_scan sick_mrs_1xxx.launch
```
4. Launch rviz
```bash
rviz rviz
```
5. Select laser as fixed frame and add a new PointCloud decoder subscribed at topic cloud.


### Sensor data TODO
#### Topics

#### Messages
