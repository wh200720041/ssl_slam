# FLOAM - SSL
## Fast LOAM (Lidar Odometry And Mapping) for solid state lidar (Intel Realsense L515 as an example)

This code is modified from [FLOAM](https://github.com/wh200720041/floam) 

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

This project is under development, and currently only L515 was tested. Because for solid-state lidar, different company may have different configuration. If you would like to try other senseors, 
you may test with current version, or email me your sensor brand, configuration as well as the rosbag recording. I will have a quick evaluation when free. 

This project is still under construction because I don't have time to test on robot pltform to further evaluate the performance. You are welcomed to leave feedback of test result on your own platform. Thanks in advance.

## 1. Solid-State Lidar Sensor Example
<p align='center'>
<img width="65%" src="/img/realsense_L515.jpg"/>
</p>
Sensor Image

<p align='center'>
<img width="65%" src="/img/realsense_L515_description.jpg"/>
</p>

## 2. Evaluation
TODO 


## 3. Prerequisites
### 3.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 3.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 3.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 3.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 4. Build 
### 4.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/floam_ssl.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
### 4.2 Dataset
Currently there are no public dataset for use and I don't have solid-state lidar on hand. 
You need to use your own sensor or record your own rosbag.

### 4.3 Launch ROS
```
    roslaunch floam_ssl floam_ssl.launch
```
if you would like to create the map at the same time, you can run (more cpu cost)
```
    roslaunch floam floam_mapping.launch
```
If the mapping process is slow, you may wish to change the rosbag speed by replacing "--clock -r 0.5" with "--clock -r 0.2" in your launch file, or you can change the map publish frequency manually (default is 10 Hz)

