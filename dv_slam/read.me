# Testing slam architecture
[hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
[lego_loam](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
[cartograhper](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

## Install

### Create catkin directory
```bash
mkdir catkin_ws
```

### hdl_graph_slam
```bash
cd catkin_ws

# for melodic
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o
# for noetic
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o

cd src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
https://github.com/koide3/hdl_graph_slam.git

cd ..
catkin_make -j1
```

### lego_loam
```bash
cd catkin_ws
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
unzip gtsam.zip
cd gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
sudo make install

cd src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1
```


### cartographer
```bash
sudo apt-get update

# for noetic
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
# for older distributions
sudo apt-get install -y python-wstool python-rosdep ninja-build stow

cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

sudo rosdep init # Command 'sudo rosdep init' can print error. This error can be ignored.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

src/cartographer/scripts/install_abseil.sh

catkin_make_isolated --install --use-ninja
```

## Rosbag
We will use rosbag to test architectures, which you can find at this [link](https://gitlab.com/eufs/datasets?fbclid=IwAR1vnGjB2k1ITUhf2fRoKBvFnC1FsZn7_6QYlg1xep08GmN4RJaSDyr5xOI#fsai). We download the rosbag located under the text FS-AI 2018 Datasets.


## Launch slam
### hdl_graph_slam
```bash
roscore

rosparam set use_sim_time true

roslaunch hdl_graph_slam hdl_graph_slam_501.launch

rviz -d hdl_graph_slam.rviz

rosbag play --clock <bag_name>.bag
```

### lego_loam
```bash
roslaunch lego_loam run.launch

rosbag play --clock <bag_name>.bag --topic /velodyne_points /imu/data
```

### cartographer TODO
