cmake_minimum_required(VERSION 2.8.3)
project(get_sensor_data)

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

#for getKinectdata.cpp
add_executable(getKinectdata src/getKinectdata.cpp)
target_link_libraries(getKinectdata ${catkin_LIBRARIES})
target_link_libraries (getKinectdata ${PCL_LIBRARIES})
target_link_libraries(getKinectdata ${OpenCV_LIBRARIES})

#for visualization.cpp
add_executable(visualization src/visualization.cpp)
target_link_libraries (visualization ${PCL_LIBRARIES})

#for clustering.cpp
add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES})
target_link_libraries (clustering ${PCL_LIBRARIES})
target_link_libraries(clustering ${OpenCV_LIBRARIES})

