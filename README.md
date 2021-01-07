# FLOAM - SSL
## Fast LOAM (Lidar Odometry And Mapping) for solid state lidar (Intel Realsense L515 as an example)

This code is modified from [FLOAM](https://github.com/wh200720041/floam) 

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

Running speed: 20 Hz on Intel NUC, 30 Hz on PC

## 1. Solid-State Lidar Sensor Example
### 1.1 Scene reconstruction
<p align='center'>
<a href="https://youtu.be/Ox7yDx6JslQ">
<img width="65%" src="/img/3D_reconstruction.gif"/>
</a>
</p>

### 1.2 SFM building example
<p align='center'>
<img width="65%" src="/img/3D_reconstruction.png"/>
</p>

### 1.3 Localization and Mapping with L515
<p align='center'>
<a href="https://youtu.be/G5aruo2bSxc">
<img width="65%" src="/img/3D_SLAM.gif"/>
</a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4 **OctoMap**
Follow [OctoMap Installation](http://wiki.ros.org/octomap).

```bash
$ sudo apt install ros-melodic-octomap*
```

### 2.5. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 3. Build 
### 3.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/floam_ssl.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 3.2 Download test rosbag
You may download our [recorded data](https://drive.google.com/file/d/1ZY6Kp5MEGBRoSP6cU2YjNTg5qsy8wNIe/view?usp=sharing) if you dont have realsense L515, and by defult the file should be under home/user/Downloads

### 3.3 Launch ROS
if you would like to create the map at the same time, you can run 
```
    roslaunch floam floam_ssl_mapping.launch
```
or create probability map 
```
    roslaunch floam floam_ssl_octo_mapping.launch
```

if only localization is required, you may refer to run
```
    roslaunch floam_ssl floam_ssl.launch
```

## 4. Sensor Setup
If you have new Realsense L515 sensor, you may follow the below setup instructions


### 4.1 L515
<p align='center'>
<img width="35%" src="/img/realsense_L515.jpg"/>
</p>

### 4.2 Librealsense
Follow [Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### 4.3 Realsense_ros
Copy realsense_ros package to your catkin folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    catkin_make
```

### 4.4 Change parameter for L515
Change the parameter setting for L515
```
 sudo gedit ~/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch 
```
search for the below argument and change default setting to below setting
```
<arg name="color_width"         default="1280"/>
<arg name="color_height"        default="720"/>
<arg name="filters"             default="pointcloud"/>
```

### 4.5 Launch ROS
```
    roslaunch floam floam_ssl_L515.launch
```
