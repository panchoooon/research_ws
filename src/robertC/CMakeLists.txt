cmake_minimum_required(VERSION 2.8.3)
project(robertC)


find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  OpenCV REQUIRED
  image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#for 2-1.cpp
add_executable(2-1 src/2-1.cpp)

#for 4-5.cpp
add_executable(4-5 src/4-5.cpp)



