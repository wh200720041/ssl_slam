cmake_minimum_required(VERSION 2.8.3)
project(ssl_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)



add_executable(ssl_slam_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(ssl_slam_laser_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ssl_slam_odom_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(ssl_slam_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ssl_slam_laser_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
target_link_libraries(ssl_slam_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ssl_slam_octo_mapping_node src/octoMappingNode.cpp src/octoMappingClass.cpp src/lidar.cpp)
target_link_libraries(ssl_slam_octo_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OCTOMAP_LIBRARIES})